#include "lane_detector.hpp"
#include "constants.hpp"
#include <iostream>
#include <numeric>
#include <cmath>

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

    // 가장 왼쪽 차선과 가장 오른쪽 차선 블롭 선택
    std::vector<std::vector<int>> result;
    if (!blobs.empty()) {
        auto left_it = std::min_element(blobs.begin(), blobs.end(), [](const auto& a, const auto& b) {
            return a.front() < b.front();
        });
        auto right_it = std::max_element(blobs.begin(), blobs.end(), [](const auto& a, const auto& b) {
            return a.back() < b.back();
        });
        int mid = width / 2;
        // left_it이 화면 오른쪽에 있거나 right_it이 화면 왼쪽에 있으면 제외
        if (left_it->front() < mid) {
            result.push_back(*left_it);
        }
        if (right_it->back() > mid) {
            result.push_back(*right_it);
        }
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

    // ROI 설정 부분 주석 처리 (마스크 비활성화)
    // cv::Mat roi_mask = createTrapezoidMask(height, width);  // ROI 생성 (차선 추적)
    cv::Mat valid_mask = (v >= VALID_V_MIN); // ROI 마스크 없이 전체 영역을 사용

    // 흰색 차선 마스크 생성
    cv::Mat white_mask = (s < WHITE_S_MAX) & (v >= WHITE_V_MIN) & valid_mask;

    // 노란색 차선 마스크 생성
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);

    vis_out = frame.clone();
    std::vector<int> target_rows = { static_cast<int>(height * 0.35f), static_cast<int>(height * 0.65f) };
    std::vector<cv::Point> lane_points;

    for (int y : target_rows) {
        const uchar* row_ptr = (WHITE_LINE_DRIVE ? white_mask.ptr<uchar>(y) : yellow_mask.ptr<uchar>(y));
        auto blobs = findBlobs(row_ptr, width);

        // 오른쪽 차선만 추적
        if (follow_right_lane) {
            if (blobs.size() == 1) {
                int x = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
                int x_right = (x > center_x) ? x : center_x + 50;  // 오른쪽 차선만 추적
                lane_points.emplace_back(x_right, y);
                cv::circle(vis_out, cv::Point(x_right, y), 3, cv::Scalar(0, 255, 255), -1);
            }
        } 
        // 왼쪽 차선만 추적
        else if (follow_left_lane) {
            if (blobs.size() == 1) {
                int x = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
                int x_left = (x < center_x) ? x : center_x - 50;  // 왼쪽 차선만 추적
                lane_points.emplace_back(x_left, y);
                cv::circle(vis_out, cv::Point(x_left, y), 3, cv::Scalar(0, 255, 255), -1);
            }
        }
    }

    // 오프셋 및 차선 교점 가중 합산
    float offset_sum = 0;
    for (const auto& pt : lane_points)
        offset_sum += (pt.x - center_x);
    float avg_offset = offset_sum / lane_points.size();

    // 최종 제어 신호
    float control = avg_offset * AVG_PARAM;

    yellow_pixel_count_ = cv::countNonZero(yellow_mask);  // 노란색 차선의 픽셀 수
    return static_cast<int>(control);
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
    std::vector<cv::Point> pts = {
        {x_center - long_half, height}, 
        {x_center + long_half, height}, 
        {x_center + short_half, y_top}, 
        {x_center - short_half, y_top}};
    cv::fillConvexPoly(mask, pts, 255);
    if (ROI_REMOVE_LEFT)
        cv::rectangle(mask, 
            cv::Point(0, 0), 
            cv::Point(ROI_REMOVE_LEFT_X_THRESHOLD, height), 
            0, 
            cv::FILLED);
    return mask;
}
