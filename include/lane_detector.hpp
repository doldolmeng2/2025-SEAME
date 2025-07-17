// lane_detector.hpp
#pragma once

#include <opencv2/opencv.hpp>

class LaneDetector {
public:
    LaneDetector();

    // frame에서 차선을 검출하고, vis_out에 시각화 결과를 저장한 뒤
    // 차선 중심 오프셋(px)을 반환한다.
    int process(const cv::Mat& frame, cv::Mat& vis_out);

    // 최신 노란색 차선 픽셀 개수를 반환
    int getYellowPixelCount() const;

private:
    // HSV 이미지에서 흰색/노란색 마스크 생성
    std::pair<cv::Mat, cv::Mat> createColorMasks(const cv::Mat& hsv_img) const;

    // 한 행(row)에서 연속된 픽셀 블롭을 찾아 반환
    std::vector<std::vector<int>> findLaneBlobs(const uchar* row_ptr, int width) const;

    // 검출된 블롭에서 차선 포인트를 추출
    std::vector<cv::Point> extractLanePoints(const std::vector<std::vector<int>>& blobs,
                                         int center_x,
                                         int y) const;


    // 포인트 배열에서 평균 오프셋을 계산
    int computeOffset(const std::vector<cv::Point>& points, int center_x) const;

    int yellow_pixel_count_{0};
};
