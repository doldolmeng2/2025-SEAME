#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetector {
public:
    LaneDetector();

    // 조향각과 감지 플래그 반환
    int process(const cv::Mat& frame, cv::Mat& vis_out);

private:
    cv::Mat createTrapezoidMask(int height, int width);
    std::vector<std::vector<int>> findBlobs(const uchar* row_ptr, int width, int min_blob_size = 10);
};
