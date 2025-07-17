#include "usb_cam.hpp"
#include "constants.hpp"
#include <iostream>

using namespace Constants;

bool USBCam::init() {
    // GStreamer 파이프라인을 통해 카메라 초기화
    std::string pipeline =
        "v4l2src device=/dev/video0 ! "
        "bayer2rgb ! "
        "videoconvert ! "
        "videoscale ! video/x-raw,width=" + std::to_string(FRAME_WIDTH) + ",height=" + std::to_string(FRAME_HEIGHT) + " ! "
        "appsink";

    cap.open(pipeline, cv::CAP_GSTREAMER);  // GStreamer로 비디오 캡처 객체 초기화
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] GStreamer 파이프라인으로 카메라 열기 실패" << std::endl;
        return false;  // 카메라 초기화 실패 시 false 반환
    }

    std::cout << "[INFO] 카메라 초기화 성공" << std::endl;
    return true;  // 카메라 초기화 성공 시 true 반환
}

cv::Mat USBCam::getFrame() {
    cv::Mat frame, resized;
    cap >> frame;  // 카메라로부터 프레임 캡처

    if (frame.empty()) {
        std::cerr << "[WARN] 프레임 읽기 실패" << std::endl;
        return cv::Mat();  // 프레임이 비어 있으면 빈 행렬 반환
    }

    // 캡처한 프레임을 지정된 크기로 리사이즈
    cv::resize(frame, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
    return resized;  // 리사이즈된 프레임 반환
}
