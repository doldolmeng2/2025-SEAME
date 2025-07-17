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

    // 관심영역 마스크
    cv::Mat roi_mask = createTrapezoidMask(height, width);
    cv::Mat valid_mask = (v >= VALID_V_MIN) & roi_mask;
    cv::Mat white_mask = (s < WHITE_S_MAX) & (v >= WHITE_V_MIN) & valid_mask;
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);

    // if (VIEWER) {
    //     cv::imshow("roi_mask", roi_mask);
    //     cv::waitKey(1);
    // }

    vis_out = frame.clone();
    std::vector<int> target_rows = { static_cast<int>(height * 0.35f), static_cast<int>(height * 0.65f) };
    std::vector<cv::Point> lane_points;

    for (int y : target_rows) {
        const uchar* row_ptr = (WHITE_LINE_DRIVE ? white_mask.ptr<uchar>(y) : yellow_mask.ptr<uchar>(y));
        auto blobs = findBlobs(row_ptr, width);

        if (blobs.size() >= 2) {
            int x1 = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
            int x2 = std::accumulate(blobs[1].begin(), blobs[1].end(), 0) / blobs[1].size();
            if (x1 > x2) std::swap(x1, x2);
            lane_points.emplace_back(x1, y);
            lane_points.emplace_back(x2, y);
            cv::circle(vis_out, cv::Point(x1, y), 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(vis_out, cv::Point(x2, y), 3, cv::Scalar(0, 255, 255), -1);
        } else if (blobs.size() == 1) {
            int x = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
            // 원근감 반영한 동적 차간 간격
            float ratio = static_cast<float>(y) / static_cast<float>(height);
            int lane_gap = static_cast<int>(DEFAULT_LANE_GAP * ratio);
            int x_other = (x < center_x) ? x + lane_gap : x - lane_gap;
            lane_points.emplace_back(x, y);
            lane_points.emplace_back(x_other, y);
            cv::circle(vis_out, cv::Point(x, y), 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(vis_out, cv::Point(x_other, y), 3, cv::Scalar(0, 255, 255), -1);
        } else {
            lane_points.emplace_back(center_x - 60, y);
            lane_points.emplace_back(center_x + 60, y);
        }
    }

    // 오프셋 및 차선 교점 가중 합산
    float offset_sum = 0;
    for (const auto& pt : lane_points)
        offset_sum += (pt.x - center_x);
    float avg_offset = offset_sum / lane_points.size();

    // 좌/우 차선 선형 교점 계산
    cv::Point2f p_left1 = lane_points[0];
    cv::Point2f p_left2 = lane_points[2];
    cv::Point2f p_right1 = lane_points[1];
    cv::Point2f p_right2 = lane_points[3];
    float denom = (p_left1.x - p_left2.x) * (p_right1.y - p_right2.y)
                - (p_left1.y - p_left2.y) * (p_right1.x - p_right2.x);
    float inter_offset = 0.0f;
    if (std::abs(denom) > 1e-6f) {
        float num_x = (p_left1.x * p_left2.y - p_left1.y * p_left2.x) * (p_right1.x - p_right2.x)
                    - (p_left1.x - p_left2.x) * (p_right1.x * p_right2.y - p_right1.y * p_right2.x);
        float ix = num_x / denom;
        inter_offset = ix - center_x;
    }

    // 최종 제어 신호
    float control = avg_offset * AVG_PARAM + inter_offset * INTER_PARAM;

    // 디버그 텍스트
    cv::putText(vis_out, "avg: " + std::to_string(avg_offset), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    cv::putText(vis_out, "int: " + std::to_string(inter_offset), cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);

    yellow_pixel_count_ = cv::countNonZero(yellow_mask);
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
