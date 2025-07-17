#include "object_detector.hpp"
#include "constants.hpp"
#include <opencv2/opencv.hpp>

DetectionResults ObjectDetector::detect(const cv::Mat& frame, cv::Mat& vis_out) {
    DetectionResults results;

    // 그레이스케일 변환
    cv::Mat gray = toGrayscale(frame);

    // ROI 마스크 생성
    cv::Mat roi_mask = createRoiMask(frame.rows, frame.cols);

    // 객체 감지
    results.stopline  = detectStopLine(gray, vis_out);
    results.crosswalk = detectCrosswalk(gray, vis_out);
    results.startline = detectStartLine(gray, vis_out);

    return results;  // 결과 반환
}

cv::Mat ObjectDetector::toGrayscale(const cv::Mat& frame) const {
    cv::Mat hsv, gray(frame.size(), CV_8UC1, cv::Scalar(0));  // 초기화된 그레이스케일 이미지
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);  // BGR에서 HSV로 변환
    std::vector<cv::Mat> ch;
    cv::split(hsv, ch);
    const cv::Mat& h = ch[0];
    const cv::Mat& s = ch[1];
    const cv::Mat& v = ch[2];

    // 유효한 부분만 필터링 (밝기와 채도 기준)
    cv::Mat valid = (v >= Constants::VALID_V_MIN) & createRoiMask(frame.rows, frame.cols);
    cv::Mat white = valid & (s < Constants::WHITE_S_MAX) & (v >= Constants::WHITE_V_MIN);
    cv::Mat yellow = valid & (~white) & (h >= Constants::YELLOW_H_MIN) & (h <= Constants::YELLOW_H_MAX);

    gray.setTo(255, white);   // 흰색 영역은 흰색으로 설정
    gray.setTo(127, yellow);  // 노란색 영역은 노란색으로 설정
    return gray;  // 변환된 그레이스케일 이미지 반환
}

cv::Mat ObjectDetector::createRoiMask(int height, int width) const {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);  // 검정색 마스크 생성

    int y_top    = Constants::ROI_Y_START;  // ROI의 상단
    int y_bottom = Constants::ROI_Y_END;    // ROI의 하단
    int x_left   = Constants::REMOVE_LEFT_ROI ? Constants::REMOVE_LEFT_ROI_THRESHOLD : 0;  // 왼쪽 ROI 제거 여부
    int x_right  = width;  // 오른쪽은 전체 화면

    // 사각형 영역을 그려서 ROI 마스크 생성
    std::vector<cv::Point> pts = {
        {x_left,  y_bottom},
        {x_right, y_bottom},
        {x_right, y_top},
        {x_left,  y_top}
    };
    cv::fillConvexPoly(mask, pts, 255);  // ROI 영역을 흰색으로 설정
    return mask;  // 생성된 ROI 마스크 반환
}

bool ObjectDetector::detectStopLine(const cv::Mat& gray, cv::Mat& vis_out) const {
    int y1 = static_cast<int>(gray.rows * Constants::STOPLINE_Y1);
    int y2 = static_cast<int>(gray.rows * Constants::STOPLINE_Y2);
    cv::Mat roi = gray.rowRange(y1, y2);  // 정지선이 있을 영역 추출

    // 흰색 영역을 추출하여 마스크 생성
    cv::Mat mask = (Constants::WHITE_LINE_DRIVE ? (roi == 255) : (roi == 127));
    cv::Mat labels, stats, centroids;
    int num = cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    int maxArea = 0, bestIdx = -1;
    // 최대 영역을 찾아 정지선으로 판단
    for (int i = 1; i < num; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > maxArea) {
            cv::Mat comp = (labels == i);
            int transitions = 0;
            for (int row = 0; row < comp.rows; ++row) {
                const uchar* ptr = comp.ptr<uchar>(row);
                for (int col = 1; col < comp.cols; ++col)
                    if (ptr[col] != ptr[col - 1]) ++transitions;
            }
            if (transitions < 15) {  // 불필요한 변화를 줄이기 위한 필터링
                maxArea = area;
                bestIdx = i;
            }
        }
    }

    // 감지된 정지선 영역이 일정 비율 이상일 경우 정지선으로 판단
    float ratio = static_cast<float>(maxArea) / (roi.rows * roi.cols);
    if (bestIdx >= 0 && ratio >= Constants::STOPLINE_THRESHOLD) {
        if (!vis_out.empty()) {
            int x = stats.at<int>(bestIdx, cv::CC_STAT_LEFT);
            int w = stats.at<int>(bestIdx, cv::CC_STAT_WIDTH);
            cv::rectangle(vis_out, cv::Rect(x, y1, w, stats.at<int>(bestIdx, cv::CC_STAT_HEIGHT)), cv::Scalar(255,0,0), 2);  // 시각화
            cv::putText(vis_out, "Stop Line", {10, y1-10}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,0,0}, 2);
        }
        return true;  // 정지선 감지됨
    }
    return false;  // 정지선 미감지
}

bool ObjectDetector::detectCrosswalk(const cv::Mat& gray, cv::Mat& vis_out) const {
    int h = gray.rows, w = gray.cols;
    int y1 = static_cast<int>(h * Constants::CROSSWALK_Y1);
    int y2 = static_cast<int>(h * Constants::CROSSWALK_Y2);
    int x1 = static_cast<int>(w * Constants::CROSSWALK_X1);
    int x2 = static_cast<int>(w * Constants::CROSSWALK_X2);
    cv::Mat roi = gray(cv::Range(y1, y2), cv::Range(x1, x2));  // 횡단보도 영역 추출

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int count = 0;
    // 횡단보도 직사각형 영역을 찾음
    for (auto& cnt : contours) {
        cv::Rect r = cv::boundingRect(cnt);
        if (r.height > Constants::CROSSWALK_RECT_HEIGHT && r.width < Constants::CROSSWALK_RECT_WIDTH) {
            ++count;
            if (!vis_out.empty())
                cv::rectangle(vis_out, r + cv::Point(x1, y1), cv::Scalar(0,255,0), 1);  // 시각화
        }
    }

    // 일정 횟수 이상 횡단보도가 감지되면 횡단보도로 판단
    if (count >= Constants::CROSSWALK_COUNT_THRESHOLD) {
        if (!vis_out.empty())
            cv::putText(vis_out, "Crosswalk", {x1+10, y1-10}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);
        return true;  // 횡단보도 감지됨
    }
    return false;  // 횡단보도 미감지
}

bool ObjectDetector::detectStartLine(const cv::Mat& gray, cv::Mat& vis_out) const {
    int h = gray.rows, w = gray.cols;
    int y1 = static_cast<int>(h * Constants::STARTLINE_Y1);
    int y2 = static_cast<int>(h * Constants::STARTLINE_Y2);
    int x1 = static_cast<int>(w * Constants::STARTLINE_X1);
    int x2 = static_cast<int>(w * Constants::STARTLINE_X2);
    cv::Mat roi = gray(cv::Range(y1, y2), cv::Range(x1, x2));  // 출발선 영역 추출

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(roi, corners,
                            Constants::GFT_MAX_CORNERS,
                            Constants::GFT_QUALITY_LEVEL,
                            Constants::GFT_MIN_DISTANCE);  // 좋은 특징점 추출
    if (!vis_out.empty()) {
        for (auto& pt : corners) {
            cv::circle(vis_out, cv::Point(static_cast<int>(pt.x) + x1, static_cast<int>(pt.y) + y1), 2, {0,255,255}, -1);  // 시각화
        }
    }

    // 일정 개수 이상의 특징점이 감지되면 출발선으로 판단
    if (static_cast<int>(corners.size()) >= Constants::STARTLINE_THRESHOLD) {
        if (!vis_out.empty())
            cv::putText(vis_out, "Start Line", {x1+10, y1+30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,255}, 2);
        return true;  // 출발선 감지됨
    }
    return false;  // 출발선 미감지
}
