// video_recorder.cpp
#include "video_recorder.hpp"
#include <iostream>

VideoRecorder::VideoRecorder() : initialized(false) {}

VideoRecorder::~VideoRecorder() {
    release();
}

bool VideoRecorder::init(const std::string& filename, int width, int height, double fps) {
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    initialized = writer.open(filename, fourcc, fps, cv::Size(width, height));

    if (!initialized) {
        std::cerr << "[ERROR] 비디오 저장 초기화 실패: " << filename << std::endl;
        return false;
    }

    std::cout << "[INFO] 비디오 저장 시작: " << filename << std::endl;
    return true;
}

void VideoRecorder::write(const cv::Mat& frame) {
    if (initialized) {
        writer.write(frame);
    }
}

void VideoRecorder::release() {
    if (initialized) {
        writer.release();
        initialized = false;
        std::cout << "[INFO] 비디오 저장 종료" << std::endl;
    }
}
