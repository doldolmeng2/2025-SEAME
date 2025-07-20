// control.cpp
#include "control.hpp"
#include "constants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>

using namespace std::chrono;  // 시간 관련 유틸 사용

// 생성자: 
Controller::Controller()
    : drive_state_(DriveState::DRIVE),   // 초기 주행 상태 설정
      steering_(-0.0f),                 // 기본 스티어링 초기값
      throttle_(0.0f)                    // 기본 스로틀 초기값
{}

// 소멸자: 모터 정지,
Controller::~Controller() {
    // 모터를 완전히 중지시켜 안전 확보
}

// cross_offset: 차선 중심 대비 오프셋
void Controller::update(int cross_offset) {
    try {
        throttle_ = computeThrottle(cross_offset);
        steering_ = computeSteering(cross_offset);
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 제어 실패: " << e.what() << "\n";
    }
}

// computeSteering: 오프셋 기반 조향 계산 (비례 제어 + 범위 제한)
float Controller::computeSteering(int offset) const {
    return std::clamp(STEERING_OFFSET + STEERING_KP * offset, -0.7f, 0.7f);
}

// computeThrottle: 현재 고정 스로틀 반환 (추후 속도 제어 로직 보완 가능)
float Controller::computeThrottle(int /*offset*/) const {
    return BASE_THROTTLE;
}
