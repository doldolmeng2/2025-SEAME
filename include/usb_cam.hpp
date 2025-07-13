#pragma once

#include <opencv2/opencv.hpp>

class USBCam {
public:
    bool init();             // 카메라 초기화
    cv::Mat getFrame();      // 프레임 가져오기 (320x200으로 리사이즈)

private:
    cv::VideoCapture cap;    // OpenCV GStreamer 캡처 객체
};
