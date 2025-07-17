#ifndef USB_CAM_HPP
#define USB_CAM_HPP

#include <opencv2/opencv.hpp>
#include "constants.hpp"

class USBCam {
public:
    USBCam() = default;

    // 카메라 초기화 함수
    bool init();

    // 카메라로부터 프레임을 읽어오는 함수
    cv::Mat getFrame();

private:
    cv::VideoCapture cap;  // 카메라 캡처 객체
};

#endif  // USB_CAM_HPP
