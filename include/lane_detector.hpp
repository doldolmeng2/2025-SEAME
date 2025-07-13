// lane_detector.hpp
#ifndef LANE_DETECTOR_HPP
#define LANE_DETECTOR_HPP

#include <opencv2/opencv.hpp>

class LaneDetector {
public:
    LaneDetector() = default;
    int process(const cv::Mat& frame); // 조향값 계산 후 반환
};

#endif // LANE_DETECTOR_HPP