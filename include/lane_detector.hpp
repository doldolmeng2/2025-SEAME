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
    cv::Vec2f slidingWindowDual(const cv::Mat& binary, const std::string& side, bool& valid);
    cv::Point computeIntersection(const cv::Vec2f& poly1, const cv::Vec2f& poly2, bool& valid);

    cv::Vec2f prev_poly_left_, prev_poly_right_;
    bool has_prev_left_ = false, has_prev_right_ = false;
};
