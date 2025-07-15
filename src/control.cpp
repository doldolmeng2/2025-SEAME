// control.cpp
#include "control.hpp"
#include "constants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <pybind11/embed.h>

namespace py = pybind11;
using namespace std::chrono;

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
      manual_mode_(false) // 기본은 자동 모드
{
    try {
        py::initialize_interpreter();
        py::module_ piracer_module = py::module_::import("piracer.vehicles");
        impl_->piracer_ = piracer_module.attr("PiRacerPro")();

        py::module_ gamepad_module = py::module_::import("piracer.gamepads");
        impl_->gamepad_ = gamepad_module.attr("ShanWanGamepad")();

        std::cout << "[INFO] Python PiracerPro 및 Gamepad 객체 생성 완료" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 초기화 실패: " << e.what() << std::endl;
    }
}

Controller::~Controller() {
    try {
        if (impl_ && impl_->piracer_) {
            impl_->piracer_.attr("set_throttle_percent")(0.0f);
            impl_->piracer_.attr("set_steering_percent")(0.0f);
            std::cout << "[INFO] 종료 전 모터 정지 명령 전송" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 종료 시 모터 정지 실패: " << e.what() << std::endl;
    }

    delete impl_;
    py::finalize_interpreter();
}

void Controller::update(bool stop_line, bool crosswalk, bool start_line, int cross_offset) {
    try {
        py::object data = impl_->gamepad_.attr("read_data")();
        auto a_pressed = py::bool_(data.attr("button_a"));
        auto b_pressed = py::bool_(data.attr("button_b"));

        if (a_pressed) manual_mode_ = true;
        if (b_pressed) manual_mode_ = false;

        if (manual_mode_) {
            float manual_throttle = data.attr("analog_stick_right").attr("y").cast<float>() * 0.5f;
            float manual_steering = data.attr("analog_stick_left").attr("x").cast<float>();

            throttle_ = manual_throttle;
            steering_ = manual_steering;
        } else {
            if (drive_state_ == DriveState::DRIVE) {
                if (stop_line && crosswalk) {
                    drive_state_ = DriveState::WAIT_AFTER_CROSSWALK;
                    wait_start_time_ = steady_clock::now();
                    std::cout << "[INFO] 정지선 + 횡단보도 감지됨 → " << WAIT_SECONDS << "초 정지 시작\n";
                } else if (start_line) {
                    drive_state_ = DriveState::STOP_AT_START_LINE;
                    std::cout << "[INFO] 출발선 감지됨 → 정지\n";
                }
            } else if (drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                auto elapsed = duration_cast<seconds>(steady_clock::now() - wait_start_time_).count();
                if (elapsed >= WAIT_SECONDS) {
                    drive_state_ = DriveState::DRIVE;
                    std::cout << "[INFO] " << WAIT_SECONDS << "초 경과 → 주행 재개\n";
                }
            }

            if (drive_state_ == DriveState::STOP_AT_START_LINE || drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                throttle_ = 0.0f;
            } else {
                throttle_ = computeThrottle(cross_offset);
            }

            steering_ = computeSteering(cross_offset);
        }

        impl_->piracer_.attr("set_steering_percent")(steering_);
        impl_->piracer_.attr("set_throttle_percent")(throttle_);

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 제어 실패: " << e.what() << std::endl;
    }

    std::cout << "[제어 출력] 모드: " << (manual_mode_ ? "수동" : "자동") << " | 상태: ";
    switch (drive_state_) {
        case DriveState::DRIVE:
            std::cout << "주행";
            break;
        case DriveState::WAIT_AFTER_CROSSWALK:
            std::cout << "횡단보도 대기";
            break;
        case DriveState::STOP_AT_START_LINE:
            std::cout << "출발선 정지";
            break;
    }

    std::cout << " | cross_offset: " << cross_offset
              << " | steering: " << steering_
              << " | throttle: " << throttle_ << "\n";
}

float Controller::computeSteering(int offset) const {
    float k = 0.001f;  // 조향 민감도
    return std::clamp(-0.25f + k * offset, -0.7f, 0.7f);
}

float Controller::computeThrottle(int offset) const {
    float base_throttle = 0.25f;
    float reduction = std::min(0.2f, std::abs(offset) * 0.0005f);
    return std::clamp(base_throttle - reduction, 0.0f, 0.8f);
}
