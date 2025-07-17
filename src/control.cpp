// control.cpp
#include "control.hpp"
#include <pybind11/embed.h>
namespace py = pybind11;

// Impl 구조체의 정의
struct Controller::Impl {
    py::object piracer_;   // PiRacerPro Python 객체
    py::object gamepad_;   // ShanWanGamepad Python 객체
};

Controller::Controller()
    : phase_(DrivePhase::START),
      phaseStartTime_(steady_clock::now()),
      firstStopPassed_(false),
      manualMode_(true),
      prevError_(0.0f),
      integral_(0.0f),
      steering_(-0.25f),
      throttle_(0.0f)
{
    // Impl 구조체 초기화
    impl_ = std::make_unique<Impl>();

    // Python 인터프리터 초기화 및 객체 생성
    try {
        py::initialize_interpreter();

        // PiRacerPro 객체 생성 (파이썬 모듈 piracer.vehicles에서 가져옴)
        auto piracer_module = py::module_::import("piracer.vehicles");
        impl_->piracer_ = piracer_module.attr("PiRacerPro")();

        // ShanWanGamepad 객체 생성 (파이썬 모듈 piracer.gamepads에서 가져옴)
        auto gamepad_module = py::module_::import("piracer.gamepads");
        impl_->gamepad_ = gamepad_module.attr("ShanWanGamepad")();

        std::cout << "[INFO] Python PiracerPro 및 ShanWanGamepad 생성 완료\n";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 객체 생성 실패: " << e.what() << "\n";
    }

    // 기본 파라미터 설정
    kp_           = Constants::STEERING_KP;
    ki_           = Constants::STEERING_KI;
    kd_           = Constants::STEERING_KD;
    throttle_kp_  = Constants::THROTTLE_KP;
    baseSteering_ = Constants::STEERING_OFFSET;
    maxError_     = static_cast<float>(Constants::DEFAULT_LANE_GAP);
}

void Controller::setManualMode(bool manual) {
    manualMode_ = manual;
}

void Controller::update(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount) {
    if (manualMode_) {
        // 수동 모드에서는 Python 객체와 상호작용 없이 steering/throttle 조정
        impl_->piracer_.attr("set_steering")(steering_);
        impl_->piracer_.attr("set_throttle")(throttle_);
        return;
    } else {
        handleAutomatic(stopLine, crosswalk, startLine, laneOffset, yellowCount);
        
        // 자동 모드에서는 Python 객체와 상호작용하여 steering/throttle 조정
        impl_->piracer_.attr("set_steering")(steering_);
        impl_->piracer_.attr("set_throttle")(throttle_);
    }
}

void Controller::handleAutomatic(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount) {
    auto now = steady_clock::now();
    int error = laneOffset - Constants::DEFAULT_LANE_GAP;

    // 페이즈별 베이스 스로틀 결정
    float baseTh = Constants::BASE_THROTTLE;
    switch (phase_) {
        case DrivePhase::START:
        case DrivePhase::CENTRE_WHITE:
        case DrivePhase::STOP_CROSSWALK:
        case DrivePhase::RIGHT_WHITE:
            baseTh = Constants::BASE_THROTTLE_LOW;
            break;
        case DrivePhase::CENTRE_YELLOW:
        case DrivePhase::LEFT_YELLOW:
            baseTh = Constants::BASE_THROTTLE_YELLOW;
            break;
        default:
            break;
    }

    switch (phase_) {
        case DrivePhase::START:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::CENTER);
            steering_ = computeSteering(error);
            throttle_ = computeThrottle(error, baseTh);
            transitionTo(DrivePhase::CENTRE_WHITE);
            break;

        case DrivePhase::CENTRE_WHITE:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::CENTER);
            steering_ = computeSteering(error);
            throttle_ = computeThrottle(error, baseTh);
            if (crosswalk)
                transitionTo(DrivePhase::STOP_CROSSWALK);
            break;

        case DrivePhase::STOP_CROSSWALK:
            Constants::WHITE_LINE_DRIVE = true;
            throttle_ = 0.0f;
            if (duration_cast<seconds>(now - phaseStartTime_).count() >= 3)
                transitionTo(DrivePhase::RIGHT_WHITE);
            break;

        case DrivePhase::RIGHT_WHITE:
            Constants::WHITE_LINE_DRIVE = true;
            firstStopPassed_ = true;
            setMode(Constants::FollowLaneMode::RIGHT);
            steering_ = computeSteering(error);
            throttle_ = computeThrottle(error, baseTh);
            if (stopLine && firstStopPassed_)
                transitionTo(DrivePhase::CENTRE_YELLOW);
            break;

        case DrivePhase::CENTRE_YELLOW:
            Constants::WHITE_LINE_DRIVE = false;
            setMode(Constants::FollowLaneMode::CENTER);
            steering_ = computeSteering(error);
            throttle_ = computeThrottle(error, baseTh);
            if (stopLine)
                transitionTo(DrivePhase::LEFT_YELLOW);
            break;

        case DrivePhase::LEFT_YELLOW:
            Constants::WHITE_LINE_DRIVE = false;
            setMode(Constants::FollowLaneMode::LEFT);
            steering_ = computeSteering(error);
            throttle_ = computeThrottle(error, baseTh);
            if (yellowCount < Constants::YELLOW_PIXEL_THRESHOLD)
                transitionTo(DrivePhase::RIGHT_WHITE_AFTER);
            break;

        case DrivePhase::RIGHT_WHITE_AFTER:
            Constants::WHITE_LINE_DRIVE = true;
            setMode(Constants::FollowLaneMode::RIGHT);
            steering_ = computeSteering(error);
            throttle_ = computeThrottle(error, baseTh);
            if (startLine)
                transitionTo(DrivePhase::FINISH);
            break;

        case DrivePhase::FINISH:
            throttle_ = 0.0f;
            break;
    }
}

void Controller::transitionTo(DrivePhase newPhase) {
    phase_ = newPhase;
    phaseStartTime_ = steady_clock::now();
}

void Controller::setMode(Constants::FollowLaneMode mode) {
    Constants::follow_lane_mode = mode;
}

float Controller::computeSteering(int error) {
    float p = kp_ * error;
    integral_ += error;
    float i = ki_ * integral_;
    float d = kd_ * (error - prevError_);
    prevError_ = error;

    float steer = p + i + d + baseSteering_;
    if (Constants::follow_lane_mode == Constants::FollowLaneMode::RIGHT)
        steer += Constants::lane_follow_steering_bias;
    else if (Constants::follow_lane_mode == Constants::FollowLaneMode::LEFT)
        steer -= Constants::lane_follow_steering_bias;

    return std::clamp(steer, -0.7f, 0.7f);
}

float Controller::computeThrottle(int error) {
    return computeThrottle(error, Constants::BASE_THROTTLE);
}

float Controller::computeThrottle(int error, float baseThrottle) {
    float t = throttle_kp_ * std::abs(error) + baseThrottle;
    return std::clamp(t, -Constants::MAX_THROTTLE, Constants::MAX_THROTTLE);
}
