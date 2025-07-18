#include "control.hpp"
#include <pybind11/embed.h>  // Python 인터프리터를 임베드하기 위한 헤더
#include <iostream>  // std::cout, std::cerr

Controller::Controller()
    : phase_(Constants::DrivePhase::START),               // 초기 주행 상태 설정
      phaseStartTime_(std::chrono::steady_clock::now()),   // 상태 시작 시간 초기화
      firstStopPassed_(false),                             // 첫 번째 정지선 통과 여부
      manualMode_(true),                                   // 수동 모드로 초기화
      prevError_(0.0f),                                    // 이전 오차 값 초기화
      integral_(0.0f),                                     // 적분 값 초기화
      steering_(-0.25f),                                   // 기본 조향 값
      throttle_(0.0f)                                      // 기본 스로틀 값
{
    py::gil_scoped_acquire gil;
    impl_ = std::make_unique<Impl>();  // Impl 객체 초기화

    try {
        // py::initialize_interpreter();  // Python 인터프리터 초기화

        // PiRacerPro 객체 생성 (파이썬 모듈에서 가져옴)
        auto piracer_module = py::module_::import("piracer.vehicles");
        impl_->piracer_ = piracer_module.attr("PiRacerPro")();

        // ShanWanGamepad 객체 생성 (파이썬 모듈에서 가져옴)
        auto gamepad_module = py::module_::import("piracer.gamepads");
        impl_->gamepad_ = gamepad_module.attr("ShanWanGamepad")();

        std::cout << "[INFO] Python PiracerPro 및 ShanWanGamepad 생성 완료\n";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 객체 생성 실패: " << e.what() << "\n";
    }

    // 기본 PID 제어 상수 설정
    kp_           = Constants::STEERING_KP;
    ki_           = Constants::STEERING_KI;
    kd_           = Constants::STEERING_KD;
    throttle_kp_  = Constants::THROTTLE_KP;
    baseSteering_ = Constants::STEERING_OFFSET;
    maxError_     = static_cast<float>(Constants::DEFAULT_LANE_GAP);
}

void Controller::setManualMode(bool manual) {
    manualMode_ = manual;
    if (!manual) {
        // 수동에서 자동으로 전환
        transitionTo(Constants::DrivePhase::START);  // 초기 자동 모드로 전환
    }
}

void Controller::update(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount) {
    py::gil_scoped_acquire gil;
    if (manualMode_) {
        // 수동 모드에서는 Python 객체와 상호작용 없이 직접 조향과 스로틀 설정
        impl_->piracer_.attr("set_steering")(steering_);
        impl_->piracer_.attr("set_throttle")(throttle_);
        return;
    } else {
        // 자동 모드에서는 조향과 스로틀을 자동으로 처리
        handleAutomatic(stopLine, crosswalk, startLine, laneOffset, yellowCount);

        // 자동 모드에서는 조향과 스로틀 값을 설정
        impl_->piracer_.attr("set_steering")(steering_);
        impl_->piracer_.attr("set_throttle")(throttle_);
    }
}

void Controller::handleAutomatic(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount) {
    auto now = std::chrono::steady_clock::now();
    int error = laneOffset - Constants::DEFAULT_LANE_GAP;

    // 기본 스로틀 값 설정
    float baseTh = Constants::BASE_THROTTLE;
    switch (phase_) {
        case Constants::DrivePhase::START:
        case Constants::DrivePhase::CENTRE_WHITE:
            baseTh = Constants::BASE_THROTTLE_LOW;
            break;
        case Constants::DrivePhase::STOP_CROSSWALK:
            baseTh = Constants::BASE_THROTTLE_LOW;
            break;
        case Constants::DrivePhase::RIGHT_WHITE:
            baseTh = Constants::BASE_THROTTLE_YELLOW;
            break;
        default:
            break;
    }

    // 조향 및 스로틀 값 계산
    steering_ = computeSteering(error);
    throttle_ = computeThrottle(error, baseTh);

    // 주행 상태에 따라 차선 추적 모드 변경 및 조향/스로틀 설정
    switch (phase_) {
        case Constants::DrivePhase::START:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::CENTER);  // 양쪽 차선 추적 모드
            transitionTo(Constants::DrivePhase::CENTRE_WHITE);
            break;

        case Constants::DrivePhase::CENTRE_WHITE:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::CENTER);
            if (crosswalk)
                transitionTo(Constants::DrivePhase::STOP_CROSSWALK);
            break;

        case Constants::DrivePhase::STOP_CROSSWALK:
            Constants::WHITE_LINE_DRIVE = true;
            throttle_ = 0.0f;  // 횡단보도 앞에서 정지
            if (std::chrono::duration_cast<std::chrono::seconds>(now - phaseStartTime_).count() >= 3)
                transitionTo(Constants::DrivePhase::RIGHT_WHITE);
            break;

        case Constants::DrivePhase::RIGHT_WHITE:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::RIGHT);  // 오른쪽 차선 추적 모드
            if (stopLine)
                transitionTo(Constants::DrivePhase::CENTRE_YELLOW);
            break;

        case Constants::DrivePhase::CENTRE_YELLOW:
            Constants::WHITE_LINE_DRIVE = false;
            setMode(Constants::FollowLaneMode::CENTER);  // 양쪽 차선 추적 모드
            if (stopLine)
                transitionTo(Constants::DrivePhase::LEFT_YELLOW);
            break;

        case Constants::DrivePhase::LEFT_YELLOW:
            Constants::WHITE_LINE_DRIVE = false;
            setMode(Constants::FollowLaneMode::LEFT);  // 왼쪽 차선 추적 모드
            if (yellowCount < Constants::YELLOW_PIXEL_THRESHOLD)
                transitionTo(Constants::DrivePhase::RIGHT_WHITE_AFTER);
            break;

        case Constants::DrivePhase::RIGHT_WHITE_AFTER:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::RIGHT);  // 오른쪽 차선 추적 모드
            if (startLine)
                transitionTo(Constants::DrivePhase::FINISH);
            break;

        case Constants::DrivePhase::FINISH:
            throttle_ = 0.0f; // 주행 종료
            break;
    }
}

void Controller::transitionTo(Constants::DrivePhase newPhase) {
    phase_ = newPhase;
    phaseStartTime_ = std::chrono::steady_clock::now();
}

void Controller::setMode(Constants::FollowLaneMode mode) {
    Constants::follow_lane_mode = mode;
}

float Controller::computeSteering(int error) {
    // PID 제어를 통해 조향 값을 계산
    float p = kp_ * error;                      // P 제어
    integral_ += error;                          // I 제어를 위한 적분 값 업데이트
    float i = ki_ * integral_;                   // I 제어
    float d = kd_ * (error - prevError_);        // D 제어
    prevError_ = error;

    float steer = p + i + d + baseSteering_;
    // 차선 모드에 따라 조향 값을 조정
    if (Constants::follow_lane_mode == Constants::FollowLaneMode::RIGHT)
        steer += Constants::lane_follow_steering_bias;
    else if (Constants::follow_lane_mode == Constants::FollowLaneMode::LEFT)
        steer -= Constants::lane_follow_steering_bias;

    return std::clamp(steer, -0.7f, 0.7f); // 조향 범위 제한
}

float Controller::computeThrottle(int error) {
    return computeThrottle(error, Constants::BASE_THROTTLE);  // 기본 스로틀 계산
}

float Controller::computeThrottle(int error, float baseThrottle) {
    // PID 제어를 통해 스로틀 값을 계산
    float t = throttle_kp_ * std::abs(error) + baseThrottle;
    return std::clamp(t, -Constants::MAX_THROTTLE, Constants::MAX_THROTTLE); // 스로틀 범위 제한
}
