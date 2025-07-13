// main.cpp
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <string>

#include "usb_cam.hpp"
#include "constants.hpp"

// 전역 공유 프레임
std::shared_ptr<cv::Mat> shared_frame = nullptr;
std::mutex frame_mutex;
std::condition_variable first_frame_cv;
bool first_frame_ready = false;
std::atomic<bool> running(true);

// 실행 모드
enum class Mode { DRIVE, RECORD, DRIVE_RECORD };
Mode current_mode = Mode::DRIVE;

int main(int argc, char** argv) {
    // 모드 설정
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

    // USB 카메라 객체 생성
    USBCam cam;
    if (!cam.init()) {
        std::cerr << "[ERROR] 카메라 초기화 실패\n";
        return 1;
    }

    // 📷 camera thread
    std::thread camera_thread([&]() {
        while (running.load()) {
            cv::Mat frame = cam.getFrame();
            auto ptr = std::make_shared<cv::Mat>(frame);

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                shared_frame = ptr;

                if (!first_frame_ready) {
                    first_frame_ready = true;
                    first_frame_cv.notify_all();
                }
            }

            // 시각화 조건부 처리
            if (VIEWER && !frame.empty()) {
                cv::imshow("Camera View", frame);
                if (cv::waitKey(1) == 27) {
                    running.store(false);
                    break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // 🚦 다른 스레드들 시작 전 첫 프레임 대기
    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        first_frame_cv.wait(lock, [] { return first_frame_ready; });
    }

    // 차후 여기에 lane_detector, object_detector, control 등 스레드 추가 예정

    camera_thread.join();

    std::cout << "[INFO] 프로그램 종료\n";
    return 0;
}
