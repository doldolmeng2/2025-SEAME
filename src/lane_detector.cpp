#include "lane_detector.hpp"
#include "constants.hpp"
#include <numeric>

using namespace Constants;

LaneDetector::LaneDetector() = default;

int LaneDetector::process(const cv::Mat& frame, cv::Mat& vis_out) {
    if (frame.empty()) {
        vis_out.release();
        return 0;  // 프레임이 비어 있으면 오프셋 0 반환
    }

    // 프레임을 HSV 색상 공간으로 변환
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 차선 마스크 생성 (흰색 차선과 노란색 차선)
    auto [white_mask, yellow_mask] = createColorMasks(hsv);

    // 시각화를 위해 원본 프레임을 복사
    vis_out = frame.clone();
    int width  = frame.cols;
    int height = frame.rows;
    int center_x = width / 2;  // 화면의 중앙 x 좌표

    // 감지할 차선의 행을 설정 (상단 35%, 하단 65%)
    std::vector<int> sample_rows = { static_cast<int>(height * 0.35f),
                                     static_cast<int>(height * 0.65f) };

    // 차선 마스크를 선택 (현재 모드에 따라 선택)
    const cv::Mat& lane_mask = (follow_lane_mode == FollowLaneMode::RIGHT) ? white_mask :
                               (follow_lane_mode == FollowLaneMode::LEFT) ? yellow_mask : 
                               (WHITE_LINE_DRIVE ? white_mask : yellow_mask);

    std::vector<cv::Point> lane_points;  // 차선 블롭을 통해 감지된 점들
    for (int y : sample_rows) {
        const uchar* row_ptr = lane_mask.ptr<uchar>(y);
        // 차선 블롭을 찾고, 차선 좌표를 추출
        auto blobs = findLaneBlobs(row_ptr, width);
        auto pts = extractLanePoints(blobs, center_x, y);
        for (auto& pt : pts) {
            cv::circle(vis_out, pt, 3, cv::Scalar(0, 255, 255), -1);  // 차선 점을 시각화
            lane_points.push_back(pt);
        }
    }

    // 차선 오프셋 계산
    int offset = computeOffset(lane_points, center_x);
    yellow_pixel_count_ = cv::countNonZero(yellow_mask);  // 노란색 차선 픽셀 수 계산
    return offset;  // 계산된 오프셋 반환
}

std::pair<cv::Mat, cv::Mat> LaneDetector::createColorMasks(const cv::Mat& hsv_img) const {
    std::vector<cv::Mat> ch;
    cv::split(hsv_img, ch);
    const auto& h = ch[0];
    const auto& s = ch[1];
    const auto& v = ch[2];

    cv::Mat valid_mask = (v >= VALID_V_MIN);
    cv::Mat white_mask = valid_mask & (s < WHITE_S_MAX) & (v >= WHITE_V_MIN);  // 흰색 차선 마스크
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);  // 노란색 차선 마스크

    return {white_mask, yellow_mask};  // 흰색과 노란색 차선 마스크 반환
}

std::vector<std::vector<int>> LaneDetector::findLaneBlobs(const uchar* row_ptr, int width) const {
    std::vector<std::vector<int>> blobs;
    std::vector<int> current;
    const int min_size = DEFAULT_LANE_GAP / 4;  // 최소 블롭 크기

    for (int x = 0; x < width; ++x) {
        if (row_ptr[x]) current.push_back(x);  // 차선이 있으면 현재 블롭에 추가
        else if (!current.empty()) {
            if (static_cast<int>(current.size()) >= min_size)
                blobs.push_back(current);  // 유효한 블롭을 저장
            current.clear();  // 새로운 블롭을 위한 초기화
        }
    }
    if (!current.empty() && static_cast<int>(current.size()) >= min_size)
        blobs.push_back(current);  // 마지막 블롭 처리

    return blobs;
}

std::vector<cv::Point> LaneDetector::extractLanePoints(
    const std::vector<std::vector<int>>& blobs,
    int center_x,
    int y) const
{
    std::vector<cv::Point> points;
    for (const auto& blob : blobs) {
        // 블롭 내 x 좌표들의 평균 계산
        int sum_x = std::accumulate(blob.begin(), blob.end(), 0);
        int avg_x = sum_x / static_cast<int>(blob.size());

        // 모드별 차선 필터링
        if (follow_lane_mode == FollowLaneMode::RIGHT) {
            if (avg_x > center_x)
                points.emplace_back(avg_x, y);
        }
        else if (follow_lane_mode == FollowLaneMode::LEFT) {
            if (avg_x < center_x)
                points.emplace_back(avg_x, y);
        }
        else {
            // 중앙 모드
            points.emplace_back(avg_x, y);
        }
    }
    return points;
}

int LaneDetector::computeOffset(const std::vector<cv::Point>& points, int center_x) const {
    if (points.empty()) return 0;  // 차선 점이 없으면 오프셋 0 반환
    double sum = 0;
    for (const auto& pt : points)
        sum += (pt.x - center_x);  // 각 차선 점과 화면 중심의 차이를 합산
    double avg = sum / points.size();  // 평균 오프셋 계산
    return static_cast<int>(avg * Constants::AVG_PARAM);  // 평균 오프셋 반환 (상수에 비례)
}

int LaneDetector::getYellowPixelCount() const {
    return yellow_pixel_count_;  // 노란색 차선 픽셀 수 반환
}
