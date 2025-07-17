// lane_detector.cpp
#include "lane_detector.hpp"
#include "constants.hpp"
#include <numeric>
using namespace Constants;

LaneDetector::LaneDetector() = default;

int LaneDetector::process(const cv::Mat& frame, cv::Mat& vis_out) {
    if (frame.empty()) {
        vis_out.release();
        return 0;
    }

    // HSV 변환 및 컬러 마스크 생성
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    auto [white_mask, yellow_mask] = createColorMasks(hsv);

    // 시각화용 복사
    vis_out = frame.clone();
    int width  = frame.cols;
    int height = frame.rows;
    int center_x = width / 2;

    // 관심 행 설정
    std::vector<int> sample_rows = { static_cast<int>(height * 0.35f),
                                     static_cast<int>(height * 0.65f) };

    // 선택된 차선 마스크
    const cv::Mat& lane_mask = (WHITE_LINE_DRIVE ? white_mask : yellow_mask);

    std::vector<cv::Point> lane_points;
    for (int y : sample_rows) {
        const uchar* row_ptr = lane_mask.ptr<uchar>(y);
        auto blobs = findLaneBlobs(row_ptr, width);
        auto pts = extractLanePoints(blobs, center_x, y);
        for (auto& pt : pts) {
            cv::circle(vis_out, pt, 3, cv::Scalar(0, 255, 255), -1);
            lane_points.push_back(pt);
        }
    }

    // 오프셋 계산 및 노란색 픽셀 수 저장
    int offset = computeOffset(lane_points, center_x);
    yellow_pixel_count_ = cv::countNonZero(yellow_mask);
    return offset;
}

std::pair<cv::Mat, cv::Mat> LaneDetector::createColorMasks(const cv::Mat& hsv_img) const {
    std::vector<cv::Mat> ch;
    cv::split(hsv_img, ch);
    const auto& h = ch[0];
    const auto& s = ch[1];
    const auto& v = ch[2];

    cv::Mat valid_mask = (v >= VALID_V_MIN);
    cv::Mat white_mask = valid_mask & (s < WHITE_S_MAX) & (v >= WHITE_V_MIN);
    cv::Mat yellow_mask = valid_mask & (~white_mask)
                          & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);
    return {white_mask, yellow_mask};
}

std::vector<std::vector<int>> LaneDetector::findLaneBlobs(const uchar* row_ptr, int width) const {
    std::vector<std::vector<int>> blobs;
    std::vector<int> current;
    const int min_size = DEFAULT_LANE_GAP / 4;  // 최소 블롭 크기

    for (int x = 0; x < width; ++x) {
        if (row_ptr[x]) current.push_back(x);
        else if (!current.empty()) {
            if (static_cast<int>(current.size()) >= min_size)
                blobs.push_back(current);
            current.clear();
        }
    }
    if (!current.empty() && static_cast<int>(current.size()) >= min_size)
        blobs.push_back(current);

    return blobs;
}

// y 행 인자를 추가한 시그니처로 가정
std::vector<cv::Point> LaneDetector::extractLanePoints(
    const std::vector<std::vector<int>>& blobs,
    int center_x,
    int y) const
{
    std::vector<cv::Point> points;
    for (const auto& blob : blobs) {
        // blob 내부 x 좌표 평균 계산
        int sum_x = std::accumulate(blob.begin(), blob.end(), 0);
        int avg_x = sum_x / static_cast<int>(blob.size());

        // 모드별 필터링
        if (Constants::follow_lane_mode == FollowLaneMode::RIGHT) {
            if (avg_x > center_x)
                points.emplace_back(avg_x, y);
        }
        else if (Constants::follow_lane_mode == FollowLaneMode::LEFT) {
            if (avg_x < center_x)
                points.emplace_back(avg_x, y);
        }
        else {
            // CENTER 모드
            points.emplace_back(avg_x, y);
        }
    }
    return points;
}

int LaneDetector::computeOffset(const std::vector<cv::Point>& points, int center_x) const {
    if (points.empty()) return 0;
    double sum = 0;
    for (const auto& pt : points)
        sum += (pt.x - center_x);
    double avg = sum / points.size();
    return static_cast<int>(avg * AVG_PARAM);
}

int LaneDetector::getYellowPixelCount() const {
    return yellow_pixel_count_;
}
