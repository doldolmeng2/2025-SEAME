#ifndef LANE_DETECTOR_HPP
#define LANE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "constants.hpp"

class LaneDetector {
public:
    LaneDetector();  // 생성자

    // 차선 감지 및 오프셋 계산
    int process(const cv::Mat& frame, cv::Mat& vis_out);

    // 노란색 차선 픽셀 수 반환
    int getYellowPixelCount() const;

private:
    // 차선 마스크 생성 (흰색과 노란색)
    std::pair<cv::Mat, cv::Mat> createColorMasks(const cv::Mat& hsv_img) const;

    // 차선 블롭을 찾는 함수
    std::vector<std::vector<int>> findLaneBlobs(const uchar* row_ptr, int width) const;

    // 차선 점을 추출하는 함수
    std::vector<cv::Point> extractLanePoints(const std::vector<std::vector<int>>& blobs, int center_x, int y) const;

    // 차선 오프셋 계산
    int computeOffset(const std::vector<cv::Point>& points, int center_x) const;

    // 현재 모드에 맞는 차선 추적
    Constants::FollowLaneMode follow_lane_mode = Constants::FollowLaneMode::CENTER;

    // 노란색 차선 픽셀 수
    int yellow_pixel_count_ = 0;
};

#endif  // LANE_DETECTOR_HPP
