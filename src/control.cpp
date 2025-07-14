// control.cpp
#include "control.hpp"
#include "constants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std::chrono;

Controller::Controller()
    : drive_state_(DriveState::DRIVE), steering_(0.0f), throttle_(0.0f) {}

void Controller::update(bool stop_line, bool crosswalk, bool start_line, int cross_offset) {
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

    // 조향/스로틀값 출력
    std::cout << "[제어 출력] 상태: ";
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
              << " | throttle: " << throttle_
              << "\n";
}

float Controller::computeSteering(int offset) const {
    // 예시: 중앙 기준 offset을 -100 ~ +100으로 보고 정규화
    float k = 0.005f;  // 조향 민감도
    return std::clamp(k * offset, -1.0f, 1.0f);
}

float Controller::computeThrottle(int offset) const {
    // 예시: offset이 작을수록 직진 성향 → 가속
    float base_throttle = 0.2f;
    float reduction = std::min(0.1f, std::abs(offset) * 0.001f);  // offset 클수록 감속
    return std::clamp(base_throttle - reduction, 0.0f, 1.0f);
}
