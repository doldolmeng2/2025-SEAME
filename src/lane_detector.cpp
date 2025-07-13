// lane_detector.cpp
#include "lane_detector.hpp"
#include <iostream>

int LaneDetector::process(const cv::Mat& frame) {
    if (frame.empty()) {
        std::cerr << "[LaneDetector] 입력 프레임이 비어있습니다." << std::endl;
        return 0;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat h = channels[0], s = channels[1], v = channels[2];

    cv::Mat white_mask = (s < 80) & (v >= 120);
    cv::Mat valid_mask = v >= 90;
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= 20) & (h <= 40);

    cv::Mat grayscale = cv::Mat::zeros(v.size(), CV_8UC1);
    grayscale.setTo(255, white_mask);
    grayscale.setTo(127, yellow_mask);

#ifdef VIEWER
    if (VIEWER) {
        cv::imshow("Lane Detection", grayscale);
        cv::waitKey(1);
    }
#endif

    // 추후 조향값 계산 로직 추가 예정
    return 0;
}
