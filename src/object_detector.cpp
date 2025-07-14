#include "object_detector.hpp"
#include "constants.hpp"
#include <iostream>
#include <numeric>

ObjectDetector::ObjectDetector() {}



int ObjectDetector::process(const cv::Mat& frame, cv::Mat& vis_out, std::vector<bool>& detection_flags) {
    if (frame.empty()) {
        std::cerr << "[ObjectDetector] 입력 프레임이 비어있습니다." << std::endl;
        return 0;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    const cv::Mat& h = channels[0];
    const cv::Mat& s = channels[1];
    const cv::Mat& v = channels[2];

    int height = frame.rows, width = frame.cols;
    detection_flags = {false, false, false}; // [정지선, 횡단보도, 출발선]

    // 관심영역 마스크 생성
    cv::Mat roi_mask = createTrapezoidMask(height, width);

    // 유효 마스크
    cv::Mat valid_mask = (v >= VALID_V_MIN) & roi_mask;
    cv::Mat white_mask = (s < WHITE_S_MAX) & (v >= WHITE_V_MIN) & valid_mask;
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);

    // 흰색=255, 노란색=127
    cv::Mat grayscale = cv::Mat::zeros(v.size(), CV_8UC1);
    grayscale.setTo(255, white_mask);
    grayscale.setTo(127, yellow_mask);

    if (VIEWER) {
        cv::imshow("Grayscale Lane", grayscale);
        cv::waitKey(1);
    }

    vis_out = frame.clone();
    // 추가 감지
    detection_flags[0] = detectStopLine(grayscale, vis_out, height, width);
    detection_flags[1] = detectCrosswalk(grayscale, vis_out, height, width);
    detection_flags[2] = detectStartLine(grayscale, vis_out, height, width);
    return 0;
}

cv::Mat ObjectDetector::createTrapezoidMask(int height, int width) {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);

    int y_top = static_cast<int>(height * 0.4);
    int x_center = width / 2;
    int long_half = width;
    int short_half = static_cast<int>(width * 0.3);

    std::vector<cv::Point> pts = {
        {x_center - long_half, height},
        {x_center + long_half, height},
        {x_center + short_half, y_top},
        {x_center - short_half, y_top}
    };

    cv::fillConvexPoly(mask, pts, 255);
    return mask;
}

bool ObjectDetector::detectStopLine(const cv::Mat& grayscale, cv::Mat& vis_out, int height, int width) {
    int y1 = static_cast<int>(height * 0.75);
    int y2 = static_cast<int>(height * 0.95);

    cv::Mat roi = grayscale.rowRange(y1, y2);
    cv::Mat white_mask = (roi == 255);

    int num_labels;
    cv::Mat labels, stats, centroids;
    num_labels = cv::connectedComponentsWithStats(white_mask, labels, stats, centroids, 8);

    int roi_area = roi.rows * roi.cols;
    int max_area = 0, max_index = -1;
    const int max_transitions = 15;

    for (int i = 1; i < num_labels; ++i) {
        cv::Mat mask = (labels == i);
        bool valid = true;

        for (int y = 0; y < mask.rows; ++y) {
            cv::Mat row = mask.row(y);
            int transitions = cv::countNonZero(row != cv::Mat(row.cols - 1, 1, row.type(), row.at<uchar>(0)));
            if (transitions >= max_transitions) {
                valid = false;
                break;
            }
        }

        if (!valid) continue;

        int area = cv::countNonZero(mask);
        if (area > max_area) {
            max_area = area;
            max_index = i;
        }
    }

    float ratio = static_cast<float>(max_area) / roi_area;
    if (ratio >= 0.35 && max_index >= 0) {
        int x = stats.at<int>(max_index, cv::CC_STAT_LEFT);
        int y = stats.at<int>(max_index, cv::CC_STAT_TOP);
        int w = stats.at<int>(max_index, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(max_index, cv::CC_STAT_HEIGHT);

        cv::rectangle(vis_out, cv::Rect(x, y + y1, w, h), cv::Scalar(255, 0, 0), 2);
        cv::putText(vis_out, "Stop Line", cv::Point(10, y1 - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
        return true;
    }

    return false;
}

bool ObjectDetector::detectCrosswalk(const cv::Mat& grayscale, cv::Mat& vis_out, int height, int width) {
    int y1 = static_cast<int>(height * 0.4);
    int y2 = static_cast<int>(height * 0.6);
    int x1 = static_cast<int>(width * 0.2);
    int x2 = static_cast<int>(width * 0.8);

    cv::Mat roi = grayscale(cv::Range(y1, y2), cv::Range(x1, x2));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int count = 0;
    for (const auto& cnt : contours) {
        cv::Rect rect = cv::boundingRect(cnt);
        if (rect.height > 20 && rect.width < 80) {
            ++count;
            cv::rectangle(vis_out, rect + cv::Point(x1, y1), cv::Scalar(0, 255, 0), 1);
        }
    }

    if (count >= 3) {
        cv::putText(vis_out, "Crosswalk", cv::Point(x1 + 10, y1 - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(vis_out, cv::Rect(x1, y1, x2 - x1, y2 - y1), cv::Scalar(0, 255, 0), 2);
        return true;
    }

    return false;
}

bool ObjectDetector::detectStartLine(const cv::Mat& grayscale, cv::Mat& vis_out, int height, int width) {
    int y1 = static_cast<int>(height * 0.5);
    int y2 = static_cast<int>(height * 0.8);
    int x1 = static_cast<int>(width * 0.2);
    int x2 = static_cast<int>(width * 0.8);

    cv::Mat roi = grayscale(cv::Range(y1, y2), cv::Range(x1, x2));
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(roi, corners, 50, 0.01, 10);

    for (const auto& pt : corners) {
        cv::circle(vis_out, cv::Point(cvRound(pt.x) + x1, cvRound(pt.y) + y1), 2, cv::Scalar(0, 255, 255), -1);
    }

    if (corners.size() >= 50) {
        cv::putText(vis_out, "Start Line", cv::Point(x1 + 10, y1 + 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        cv::rectangle(vis_out, cv::Rect(x1, y1, x2 - x1, y2 - y1), cv::Scalar(0, 255, 255), 2);
        return true;
    }

    return false;
}
