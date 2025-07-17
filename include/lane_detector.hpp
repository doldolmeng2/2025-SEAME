#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetector {
public:
    LaneDetector();

    // 조향각과 감지 플래그 반환
    int process(const cv::Mat& frame, cv::Mat& vis_out);
    int getYellowPixelCount() const;

private:
    cv::Mat createTrapezoidMask(int height, int width);
    std::vector<std::vector<int>> findBlobs(const uchar* row_ptr, int width, int min_blob_size = 5);
    cv::Mat warpBirdsEye(const cv::Mat& frame);
    // 🔽 새롭게 추가할 멤버 변수
    int prev_lane_gap_top_ = 120;    // 초기값: 대략적인 차선 간 거리
    int prev_lane_gap_bottom_ = 120;
    int yellow_pixel_count_ = 0;
};
