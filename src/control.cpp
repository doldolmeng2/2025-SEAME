// control.cpp
#include "control.hpp"
#include <algorithm>
#include <iostream>

Controller::Controller(std::shared_ptr<PiRacer> vehicle,
                       float steering_kp, float throttle_kp,
                       float max_throttle, int wait_seconds)
    : vehicle_(vehicle),
      steering_kp_(steering_kp),
      throttle_kp_(throttle_kp),
      max_throttle_(max_throttle),
      wait_duration_(wait_seconds),
      drive_state_(DriveState::DRIVE) {}

void Controller::updateAndDrive(bool stop_line_detected, bool crosswalk_detected,
                                bool start_line_detected, int cross_point_offset) {
    if (drive_state_ == DriveState::DRIVE) {
        if (stop_line_detected && crosswalk_detected) {
            drive_state_ = DriveState::WAIT_AFTER_CROSSWALK;
            wait_start_time_ = std::chrono::steady_clock::now();
            std::cout << "[INFO] 정지선 + 횡단보도 감지 → " << wait_duration_ << "초 정지 시작\n";
        } else if (start_line_detected) {
            drive_state_ = DriveState::STOP_AT_START_LINE;
            std::cout << "[INFO] 출발선 감지 → 정지\n";
        }
    } else if (drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - wait_start_time_).count();
        if (elapsed >= wait_duration_) {
            drive_state_ = DriveState::DRIVE;
            std::cout << "[INFO] 5초 경과 → 출발\n";
        }
    }

    float steering = computeSteering(cross_point_offset);
    float throttle = computeThrottle(cross_point_offset);

    vehicle_->set_steering_percent(steering);
    vehicle_->set_throttle_percent(throttle);

    std::cout << "[제어] 조향: " << steering << ", 스로틀: " << throttle << std::endl;
}

bool Controller::isStopping() const {
    return drive_state_ == DriveState::WAIT_AFTER_CROSSWALK;
}

bool Controller::isStartLineStop() const {
    return drive_state_ == DriveState::STOP_AT_START_LINE;
}

float Controller::computeSteering(int cross_point_offset) {
    float steering = -steering_kp_ * static_cast<float>(cross_point_offset);
    return std::clamp(steering, -1.0f, 1.0f);
}

float Controller::computeThrottle(int cross_point_offset) {
    if (isStopping() || isStartLineStop()) {
        return 0.0f;
    }

    float error = static_cast<float>(cross_point_offset);
    float throttle = max_throttle_ - throttle_kp_ * std::abs(error);
    return std::clamp(throttle, 0.0f, max_throttle_);
}
