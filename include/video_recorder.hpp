// video_recorder.hpp
#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class VideoRecorder {
public:
    VideoRecorder();
    ~VideoRecorder();

    bool init(const std::string& filename, int width, int height, double fps = 30.0);
    void write(const cv::Mat& frame);
    void release();

private:
    cv::VideoWriter writer;
    bool initialized;
};