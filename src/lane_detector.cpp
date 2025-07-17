#include "lane_detector.hpp"
#include "constants.hpp"
#include <iostream>
#include <numeric>

LaneDetector::LaneDetector() {}

std::vector<std::vector<int>> LaneDetector::findBlobs(const uchar* row_ptr, int width, int min_blob_size) {
    std::vector<std::vector<int>> blobs;
    std::vector<int> current_blob;

    for (int x = 0; x < width; ++x) {
        if (row_ptr[x]) {
            current_blob.push_back(x);
        } else if (!current_blob.empty()) {
            if (current_blob.size() >= min_blob_size)
                blobs.push_back(current_blob);
            current_blob.clear();
        }
    }

    // 가장 왼쪽 차선(시작 인덱스 최소)과 가장 오른쪽 차선(끝 인덱스 최대) 블롭 선택
    std::vector<std::vector<int>> result;
    if (!blobs.empty()) {
        auto left_it = std::min_element(blobs.begin(), blobs.end(), [](const auto& a, const auto& b) {
            return a.front() < b.front();
        });
        auto right_it = std::max_element(blobs.begin(), blobs.end(), [](const auto& a, const auto& b) {
            return a.back() < b.back();
        });
        result.push_back(*left_it);
        if (left_it != right_it)
            result.push_back(*right_it);
    }
    return result;
}

int LaneDetector::process(const cv::Mat& frame, cv::Mat& vis_out) {
    if (frame.empty()) {
        std::cerr << "[LaneDetector] 입력 프레임이 비어있습니다." << std::endl;
        return 0;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    const cv::Mat& h = channels[0];
    const cv::Mat& s = channels[1];
    const cv::Mat& v = channels[2];

    int height = frame.rows;
    int width = frame.cols;
    int center_x = width / 2;

    // 관심영역 마스크 생성
    cv::Mat roi_mask = createTrapezoidMask(height, width);

    // 유효 마스크 설정
    cv::Mat valid_mask = (v >= VALID_V_MIN) & roi_mask;
    cv::Mat white_mask = (s < WHITE_S_MAX) & (v >= WHITE_V_MIN) & valid_mask;
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);

    vis_out = frame.clone();
    std::vector<int> target_rows = { static_cast<int>(height * 0.4), static_cast<int>(height * 0.6) };
    std::vector<cv::Point> lane_points;
    int i = 0;

    for (int y : target_rows) {
        const uchar* row_ptr = (WHITE_LINE_DRIVE ? white_mask.ptr<uchar>(y) : yellow_mask.ptr<uchar>(y));
        auto blobs = findBlobs(row_ptr, width, MIN_BLOB_SIZE); // min_blob_size 인자 추가

        if (blobs.size() >= 2) {
            // 왼쪽, 오른쪽 차선 블롭의 평균 x 좌표 계산
            int x1 = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
            int x2 = std::accumulate(blobs[1].begin(), blobs[1].end(), 0) / blobs[1].size();
            if (x1 > x2) std::swap(x1, x2);

            // 차선 점 저장
            lane_points.emplace_back(x1, y);
            lane_points.emplace_back(x2, y);

            // 시각화: 원으로 표시
            cv::circle(vis_out, cv::Point(x1, y), 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(vis_out, cv::Point(x2, y), 3, cv::Scalar(0, 255, 255), -1);

        } else if (blobs.size() == 1) {
            int x = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
            int lane_gap = DEFAULT_LANE_GAP; // 고정값 사용
            int x_other = (x < center_x) ? x + lane_gap : x - lane_gap;

            lane_points.emplace_back(x, y);
            lane_points.emplace_back(x_other, y);

            cv::circle(vis_out, cv::Point(x, y), 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(vis_out, cv::Point(x_other, y), 3, cv::Scalar(0, 128, 255), -1);

        } else {
            // 검출 실패 시 중앙 기준 고정 좌표 사용
            lane_points.emplace_back(center_x - 60, y);
            lane_points.emplace_back(center_x + 60, y);
        }
        i++;
    }

    // 평균 오프셋 계산 및 표시
    float offset_sum = 0;
    for (const auto& pt : lane_points) offset_sum += pt.x - center_x;
    float avg_offset = offset_sum / lane_points.size();
    cv::putText(vis_out, "avg_offset: " + std::to_string(avg_offset), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);

    yellow_pixel_count_ = cv::countNonZero(yellow_mask);
    return static_cast<int>(avg_offset);
}

int LaneDetector::getYellowPixelCount() const {
    return yellow_pixel_count_;
}

cv::Mat LaneDetector::createTrapezoidMask(int height, int width) {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
    int y_top = static_cast<int>(height * Y_TOP);
    int x_center = width / 2;
    int long_half = width * LONG_HALF;
    int short_half = static_cast<int>(width * SHORT_HALF);
    std::vector<cv::Point> pts = {{x_center - long_half, height}, {x_center + long_half, height}, {x_center + short_half, y_top}, {x_center - short_half, y_top}};
    cv::fillConvexPoly(mask, pts, 255);
    if (ROI_REMOVE_LEFT) cv::rectangle(mask, cv::Point(0, 0), cv::Point(ROI_REMOVE_LEFT_X_THRESHOLD, height), 0, cv::FILLED);
    return mask;
}
