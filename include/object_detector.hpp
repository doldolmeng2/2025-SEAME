#pragma once

#include <opencv2/opencv.hpp>

struct DetectionResults {
    bool stopline   = false;
    bool crosswalk  = false;
    bool startline  = false;
};

class ObjectDetector {
public:
    ObjectDetector() = default;

    // frame에서 객체 검출 후 vis_out에 시각화, 결과는 DetectionResults로 반환
    DetectionResults detect(const cv::Mat& frame, cv::Mat& vis_out);

private:
    cv::Mat toGrayscale(const cv::Mat& frame) const;
    cv::Mat createRoiMask(int height, int width) const;

    bool detectStopLine(const cv::Mat& gray, cv::Mat& vis_out) const;
    bool detectCrosswalk(const cv::Mat& gray, cv::Mat& vis_out) const;
    bool detectStartLine(const cv::Mat& gray, cv::Mat& vis_out) const;
};
