// control.cpp
#include "control.hpp"
#include "constants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include <pybind11/embed.h>

namespace py = pybind11;
using namespace std::chrono;

bool crosswalk_flag = false;
bool crosswalk_ignore_stopline = false;
steady_clock::time_point crosswalk_resume_time;

// 내부 Impl 정의 (py::object 감싸기)
struct __attribute__((visibility("hidden"))) Controller::Impl {
    py::object piracer_;
    py::object gamepad_;
};

Controller::Controller()
    : drive_state_(DriveState::DRIVE),
      steering_(-0.25f),
      throttle_(0.0f),
      impl_(new Impl()),
      manual_mode_(false),
      manual_throttle_(0.0f),
      manual_steering_(0.0f),
      gamepad_running_(false)
{
    try {
        // 파이썬 인터프리터 초기화
        py::initialize_interpreter();

        // PiRacerPro 객체 생성
        auto piracer_module = py::module_::import("piracer.vehicles");
        impl_->piracer_ = piracer_module.attr("PiRacerPro")();

        // ShanWanGamepad 객체 생성
        auto gamepad_module = py::module_::import("piracer.gamepads");
        impl_->gamepad_ = gamepad_module.attr("ShanWanGamepad")();

        std::cout << "[INFO] Python PiracerPro 및 ShanWanGamepad 생성 완료\n";

        // 게임패드 전용 쓰레드 시작
        startGamepadThread();
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 초기화 실패: " << e.what() << "\n";
    }
}

Controller::~Controller() {
    // 게임패드 쓰레드 종료
    gamepad_running_ = false;
    if (gamepad_thread_.joinable()) {
        gamepad_thread_.join();
    }

    // 모터 정지
    try {
        if (impl_ && impl_->piracer_) {
            impl_->piracer_.attr("set_throttle_percent")(0.0f);
            impl_->piracer_.attr("set_steering_percent")(0.0f);
            std::cout << "[INFO] 종료 전 모터 정지 명령 전송\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 종료 시 모터 정지 실패: " << e.what() << "\n";
    }

    delete impl_;
    py::finalize_interpreter();
}

// ▶️ 게임패드 입력 전용 스레드
void Controller::startGamepadThread() {
    gamepad_running_ = true;
    gamepad_thread_ = std::thread([this]() {
    while (gamepad_running_) {
        try {
            // 1) GIL 획득
            py::gil_scoped_acquire gil;

            // 2) Python API 호출
            auto data = impl_->gamepad_.attr("read_data")();

                // A/B 버튼으로 모드 전환
            if (py::bool_(data.attr("button_a"))) manual_mode_ = true;
            if (py::bool_(data.attr("button_b"))) manual_mode_ = false;


            // 축값 읽기
            manual_throttle_ = data.attr("analog_stick_right")
                                    .attr("y").cast<float>() * 0.5f;
            manual_steering_ = data.attr("analog_stick_left")
                                    .attr("x").cast<float>();

            }
            catch (const std::exception& e) {
                std::cerr << "[WARN] Gamepad read failed: " << e.what() << "\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    });
}


void Controller::update(bool stop_line, bool crosswalk, bool start_line, int cross_offset, int yellow_pixel_count) {
    py::gil_scoped_acquire acquire;

    // std::cout << "[제어 출력] 모드: " << (manual_mode_ ? "수동" : "자동") << " | 상태: ";

    try {
        if (manual_mode_) {
            // 수동 모드: 조이스틱 입력 그대로 사용
            throttle_ = manual_throttle_;
            steering_ = manual_steering_;
        } else {
            // 자동 모드: 이미지 처리 기반 제어
            if (drive_state_ == DriveState::DRIVE) {
                if (crosswalk && !crosswalk_flag) {
                    crosswalk_flag = true;
                    drive_state_ = DriveState::WAIT_AFTER_CROSSWALK;
                    wait_start_time_ = steady_clock::now();
                    std::cout << "[INFO] 횡단보도 감지됨 → 대기 시작\n";
                } else if (start_line) {
                    drive_state_ = DriveState::STOP_AT_START_LINE;
                    std::cout << "[INFO] 출발선 감지 → 정지\n";
                }
            } else if (drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                auto elapsed = duration_cast<seconds>(steady_clock::now() - wait_start_time_).count();
                if (elapsed >= WAIT_SECONDS) {
                    crosswalk_ignore_stopline = true; // 정지선 무시 기간 시작
                    crosswalk_resume_time = steady_clock::now();
                    drive_state_ = DriveState::DRIVE;
                    std::cout << "[INFO] 횡단보도 정지 후 주행 재개\n";
                }
            } else if (drive_state_ == DriveState::STOP_AT_START_LINE) {
                // 아무 것도 안함 (정지 유지)
            } else if (drive_state_ == DriveState::DRIVE && crosswalk_flag) {
                if (crosswalk_ignore_stopline) {
                    // 무시 기간이 지난 경우 → 정지선 감지 다시 허용
                    auto since_resume = duration_cast<seconds>(steady_clock::now() - crosswalk_resume_time).count();
                    if (since_resume > 2) {
                        crosswalk_ignore_stopline = false;
                        std::cout << "[INFO] 정지선 감지 다시 활성화됨\n";
                    }
                    // 정지선은 무시
                } else {
                    // 정지선 감지가 다시 허용된 상태에서 정지선 감지되면 → 전환
                    if (stop_line) {
                        drive_state_ = DriveState::YELLOW_LINE_DRIVE;
                        std::cout << "[INFO] 정지선 감지됨 → 노란 차선 주행으로 전환\n";
                    }
                }
            } else if (drive_state_ == DriveState::YELLOW_LINE_DRIVE) {
                ROI_REMOVE_LEFT = true;
                WHITE_LINE_DRIVE = false;

                // 노란색 픽셀 수가 너무 적으면 일반 주행으로 복귀
                if (yellow_pixel_count < YELLOW_PIXEL_THRESHOLD) {
                    drive_state_ = DriveState::DRIVE;
                    ROI_REMOVE_LEFT = false;
                    WHITE_LINE_DRIVE = true;
                    std::cout << "[INFO] 노란 차선 사라짐 → 일반 흰색 차선 주행으로 전환\n";
                }
            }

            // 스로틀 결정
            if (drive_state_ == DriveState::STOP_AT_START_LINE ||
                drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                throttle_ = 0.0f;
            } else {
                throttle_ = computeThrottle(cross_offset);
            }

            // 스티어링 결정
            steering_ = computeSteering(cross_offset);
        }

        // PiRacerPro에 명령 전송
        impl_->piracer_.attr("set_steering_percent")(steering_);
        impl_->piracer_.attr("set_throttle_percent")(throttle_);
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 제어 실패: " << e.what() << "\n";
    }
}
//     // 상태 출력
//     switch (drive_state_) {
//         case DriveState::DRIVE: std::cout << "주행"; break;
//         case DriveState::WAIT_AFTER_CROSSWALK: std::cout << "횡단보도 대기"; break;
//         case DriveState::STOP_AT_START_LINE: std::cout << "출발선 정지"; break;
//     }
//     std::cout << " | cross_offset: " << cross_offset
//               << " | steering: " << steering_
//               << " | throttle: " << throttle_ << "\n";
// }

// computeSteering: 오프셋 기반 조향 계산 (비례 제어 + 범위 제한)
float Controller::computeSteering(int offset) const {
    static steady_clock::time_point last_debug_time = steady_clock::now() - milliseconds(100);
    auto now = steady_clock::now();
    float steering = std::clamp(-0.25f + STEERING_KP * offset, -0.7f, 0.7f);
    // 0.1초(100ms)마다 디버그 로그 출력
    if (duration_cast<milliseconds>(now - last_debug_time).count() >= 100) {
        std::cout << "[DEBUG] 조향 계산 - offset: " << offset << ", steering: " << steering << "\n";
        last_debug_time = now;
    }
    return steering;
}

float Controller::computeThrottle(int offset) const {
    float base = BASE_THROTTLE;
    return base;
    // float red   = std::min(0.2f, std::abs(offset) * 0.0005f);
    // return std::clamp(base - red, 0.0f, 0.8f);
}