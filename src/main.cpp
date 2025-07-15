// main.cpp
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <string>
#include <csignal>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "usb_cam.hpp"
#include "video_recorder.hpp"
#include "lane_detector.hpp"
#include "object_detector.hpp"  
#include "control.hpp"
#include "constants.hpp"

std::mutex frame_mutex;

std::shared_ptr<cv::Mat> shared_frame = nullptr;

std::mutex lane_mutex;
std::atomic<int> cross_point_offset = 0;

std::mutex object_mutex;
std::vector<bool> detections_flags(3, false);

std::condition_variable control_cv;
std::mutex control_mutex;
bool control_ready = false;

std::condition_variable first_frame_cv;
bool first_frame_ready = false;
std::atomic<bool> running(true);

enum class Mode { DRIVE, RECORD, DRIVE_RECORD };
Mode current_mode = Mode::DRIVE;

void signal_handler(int) {
    running = false;
    std::cout << "\n[INFO] 종료 시그널 감지됨. 프로그램 종료 중...\n";
}

std::string getTimestampedFilename(const std::string& base_dir) {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&t);

    std::ostringstream oss;
    oss << base_dir << "/output_"
        << std::setw(2) << std::setfill('0') << tm->tm_mday
        << std::setw(2) << std::setfill('0') << tm->tm_hour
        << std::setw(2) << std::setfill('0') << tm->tm_min
        << std::setw(2) << std::setfill('0') << tm->tm_sec
        << ".avi";
    return oss.str();
}

int main(int argc, char** argv) {

    try{
        load_constants("constants.json"); // constants.json에 있는 정보를 constants.hpp로 가져온다.
        std::cout << "Steering Gain: " << STEERING_KP << "\n";
    } catch (const std::exception& e){
        std::cerr << "[ERROR] 상수 로드 실패: " << e.what() << std::endl;
        return 1;
    }

    signal(SIGINT, signal_handler);

    if (argc < 2) {
        std::cerr << "[ERROR] 실행 인자를 지정해주세요: d, r, dr 중 하나\n";
        return 1;
    }

    std::string mode_arg = argv[1];
    if (mode_arg == "d") {
        current_mode = Mode::DRIVE;
    } else if (mode_arg == "r") {
        current_mode = Mode::RECORD;
    } else if (mode_arg == "dr") {
        current_mode = Mode::DRIVE_RECORD;
    } else {
        std::cerr << "[ERROR] 잘못된 모드입니다. d, r, dr 중 하나를 선택해주세요.\n";
        return 1;
    }

    std::cout << "[INFO] 선택된 모드: " << mode_arg << "\n";

    USBCam cam;
    if (!cam.init()) {
        std::cerr << "[ERROR] 카메라 초기화 실패\n";
        return 1;
    }

    VideoRecorder recorder;
    if (current_mode == Mode::RECORD || current_mode == Mode::DRIVE_RECORD) {
        std::string filename = getTimestampedFilename("/home/orda/records/avis");
        if (!recorder.init(filename, FRAME_WIDTH, FRAME_HEIGHT, 30.0)) {
            std::cerr << "[ERROR] 비디오 저장 초기화 실패\n";
            return 1;
        }
    }

    std::thread camera_thread([&]() {
        while (running.load()) {
            cv::Mat frame = cam.getFrame();
            if (frame.empty()) continue;

            auto ptr = std::make_shared<cv::Mat>(frame);

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                shared_frame = ptr;

                if (!first_frame_ready) {
                    first_frame_ready = true;
                    first_frame_cv.notify_all();
                }
            }

            if (current_mode == Mode::RECORD || current_mode == Mode::DRIVE_RECORD) {
                recorder.write(frame);
            }

            if (VIEWER) {
                // cv::imshow("Live", frame);
                if (cv::waitKey(1) == 27) {
                    running = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        first_frame_cv.wait(lock, [] { return first_frame_ready; });
    }



    std::thread lane_thread([&]() {
        LaneDetector lanedetector;
        while (running) {
            std::shared_ptr<cv::Mat> frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame = shared_frame;
            }
            if (frame && !frame->empty()) {
                cv::Mat vis_out;
                int offset = lanedetector.process(*frame, vis_out);
                {
                    std::lock_guard<std::mutex> lock(lane_mutex);
                    cross_point_offset = offset;
                }
                {
                    std::lock_guard<std::mutex> lock(control_mutex);
                    control_ready = true;
                    control_cv.notify_one();
                }
                if (VIEWER) {
                    cv::imshow("Lane", vis_out);
                    if (cv::waitKey(1) == 27) running = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::thread object_thread([&]() {
        ObjectDetector detector;
        while (running) {
            std::shared_ptr<cv::Mat> frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame = shared_frame;
            }
            if (frame && !frame->empty()) {
                cv::Mat vis_out;
                std::vector<bool> flags;
                detector.process(*frame, vis_out, flags);
                {
                    std::lock_guard<std::mutex> lock(object_mutex);
                    detections_flags = flags;
                }
                {
                    std::lock_guard<std::mutex> lock(control_mutex);
                    control_ready = true;
                    control_cv.notify_one();
                }
                if (VIEWER) {
                    cv::imshow("Objects", vis_out);
                    if (cv::waitKey(1) == 27) running = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::thread control_thread([&]() {
        Controller controller;
        while (running) {
            std::unique_lock<std::mutex> lock(control_mutex);
            control_cv.wait(lock, [] { return control_ready; });
            control_ready = false;
            lock.unlock();

            bool stop = false, cross = false, start = false;
            int offset = 0;

            {
                std::lock_guard<std::mutex> lock(lane_mutex);
                offset = cross_point_offset;
            }
            {
                std::lock_guard<std::mutex> lock(object_mutex);
                if (detections_flags.size() > 0) stop = detections_flags[0];
                if (detections_flags.size() > 1) cross = detections_flags[1];
                if (detections_flags.size() > 2) start = detections_flags[2];
            }

            controller.update(stop, cross, start, offset);
        }
    });


    camera_thread.join();
    lane_thread.join();
    object_thread.join();
    control_thread.join();

    recorder.release();

    std::cout << "[INFO] 프로그램 종료\n";
    return 0;
}