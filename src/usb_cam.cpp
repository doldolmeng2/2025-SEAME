// usb_cam.cpp (리팩터링: 클래스 기반, main 제거)
#include "usb_cam.hpp"
#include "constants.hpp"

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

bool USBCam::init() {
    std::string pipeline =
        "v4l2src device=/dev/video0 ! "
        "bayer2rgb ! "
        "videoconvert ! "
        "videoscale ! video/x-raw,width=640,height=240 ! "
        "appsink";

    cap.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] GStreamer 파이프라인으로 카메라 열기 실패" << std::endl;
        return false;
    }

    std::cout << "[INFO] 카메라 초기화 성공" << std::endl;
    return true;
}

cv::Mat USBCam::getFrame() {
    cv::Mat frame, resized;
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "[WARN] 프레임 읽기 실패" << std::endl;
        return cv::Mat();
    }
    cv::resize(frame, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
    return resized;
}
