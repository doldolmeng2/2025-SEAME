#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "constants.hpp"

// 객체 감지 결과를 담을 구조체
struct DetectionResults {
    bool stopline;   // 정지선 감지 여부
    bool crosswalk;  // 횡단보도 감지 여부
    bool startline;  // 출발선 감지 여부
};

class ObjectDetector {
public:
    ObjectDetector() = default;

    // 객체 감지 함수: 정지선, 횡단보도, 출발선 감지
    DetectionResults detect(const cv::Mat& frame, cv::Mat& vis_out);

private:
    // 그레이스케일 변환 함수
    cv::Mat toGrayscale(const cv::Mat& frame) const;

    // ROI 마스크 생성 함수
    cv::Mat createRoiMask(int height, int width) const;

    // 정지선 감지 함수
    bool detectStopLine(const cv::Mat& gray, cv::Mat& vis_out) const;

    // 횡단보도 감지 함수
    bool detectCrosswalk(const cv::Mat& gray, cv::Mat& vis_out) const;

    // 출발선 감지 함수
    bool detectStartLine(const cv::Mat& gray, cv::Mat& vis_out) const;
};

#endif  // OBJECT_DETECTOR_HPP
