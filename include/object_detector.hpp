// object_detector.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class ObjectDetector {
public:
    ObjectDetector();

    // 전처리 및 감지 실행
    int process(const cv::Mat& frame, cv::Mat& vis_out, std::vector<bool>& detection_flags);

private:
    // 영역 마스크 생성
    cv::Mat createTrapezoidMask(int height, int width);

    // 개별 객체 감지 함수
    bool detectStopLine(const cv::Mat& grayscale, cv::Mat& vis_out, int height, int width);
    bool detectCrosswalk(const cv::Mat& grayscale, cv::Mat& vis_out, int height, int width);
    bool detectStartLine(const cv::Mat& grayscale, cv::Mat& vis_out, int height, int width);
};
