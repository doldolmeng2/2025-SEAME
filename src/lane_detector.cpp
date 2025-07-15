#include "lane_detector.hpp"
#include "constants.hpp"
#include <iostream>
#include <numeric>

LaneDetector::LaneDetector() {}

std::vector<std::vector<int>> LaneDetector::findWhiteBlobs(const uchar* row_ptr, int width, int min_blob_size) {
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
    // ⬇️ 추가된 정렬 코드
    std::sort(blobs.begin(), blobs.end(), [](const auto& a, const auto& b) {
        return a.size() > b.size();
    });

    return blobs;
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

    int height = frame.rows, width = frame.cols;
    int center_x = width / 2;

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

    vis_out = frame.clone();
    
     // 흰색 마스크 기반 조향각 계산
    std::vector<int> target_rows = {
        static_cast<int>(height * 0.25),
        static_cast<int>(height * 0.75)
    };

    std::vector<cv::Point> lane_points;
    
    int i = 0;
    for (int y : target_rows) {
        const uchar* row_ptr = white_mask.ptr<uchar>(y);
        auto blobs = findWhiteBlobs(row_ptr, width);

        if (blobs.size() >= 2) {
            int x1 = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
            int x2 = std::accumulate(blobs[1].begin(), blobs[1].end(), 0) / blobs[1].size();
            if (x1 > x2) std::swap(x1, x2);

            // 🔸 두 점 사이 거리 저장
            if (i == 0) prev_lane_gap_top_ = x2 - x1;
            else        prev_lane_gap_bottom_ = x2 - x1;

            lane_points.emplace_back(x1, y);
            lane_points.emplace_back(x2, y);

            cv::circle(vis_out, cv::Point(x1, y), 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(vis_out, cv::Point(x2, y), 3, cv::Scalar(0, 255, 255), -1);
        } else if (blobs.size() == 1) {
            int x = std::accumulate(blobs[0].begin(), blobs[0].end(), 0) / blobs[0].size();
            int lane_gap = (i == 0) ? prev_lane_gap_top_ : prev_lane_gap_bottom_;

            int x_other = (x < center_x) ? x + lane_gap : x - lane_gap;

            lane_points.emplace_back(x, y);
            lane_points.emplace_back(x_other, y);

            cv::circle(vis_out, cv::Point(x, y), 3, cv::Scalar(0, 255, 255), -1);
            cv::circle(vis_out, cv::Point(x_other, y), 3, cv::Scalar(0, 128, 255), -1);
        } else {
            lane_points.emplace_back(center_x - 60, y);
            lane_points.emplace_back(center_x + 60, y);
        }
        i++;
    }

    // 중앙선 대비 오프셋 평균 계산
    float offset_sum = 0;
    for (const auto& pt : lane_points) {
        offset_sum += pt.x - center_x;
    }
    float avg_offset = offset_sum / lane_points.size();

    cv::putText(vis_out, "avg_offset: " + std::to_string(avg_offset),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);

    return static_cast<int>(avg_offset);
}

cv::Mat LaneDetector::createTrapezoidMask(int height, int width) {
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
