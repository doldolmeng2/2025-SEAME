#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <memory>  // std::unique_ptr
#include <iostream>  // std::cout, std::cerr
#include "constants.hpp"
#include <pybind11/embed.h>

namespace py = pybind11;

class Controller {
public:
    Controller();  // 생성자

    // 수동 모드와 자동 모드 전환
    void setManualMode(bool manual);

    // 자동 모드에서 차선 추적 및 객체 감지 처리
    void update(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount);

    float getSteering() const { return steering_; }
    float getThrottle() const { return throttle_; }

private:
    // 내부 구현을 위한 구조체
    struct Impl {
        py::object piracer_;   // PiRacerPro Python 객체 (PiRacer 차량 제어)
        py::object gamepad_;   // ShanWanGamepad Python 객체 (게임패드 제어)
    };

    // 주행 상태 처리
    void handleAutomatic(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount);

    // 상태 전환 함수
    void transitionTo(Constants::DrivePhase newPhase);

    // 차선 추적 모드 설정
    void setMode(Constants::FollowLaneMode mode);

    // PID 제어 관련 변수들
    float kp_, ki_, kd_; 
    float throttle_kp_;
    float steering_;
    float throttle_;
    float baseSteering_;
    float prevError_, integral_;
    float maxError_;
    
    // 주행 상태와 상태 전환을 위한 변수들
    Constants::DrivePhase phase_;
    std::chrono::steady_clock::time_point phaseStartTime_;
    bool firstStopPassed_;
    bool manualMode_;

    // Impl 객체 생성
    std::unique_ptr<Impl> impl_;  // 내부 구현을 위한 포인터

    // steering 및 throttle 계산 함수들
    float computeSteering(int error);
    float computeThrottle(int error);
    float computeThrottle(int error, float baseThrottle);
};

#endif  // CONTROL_HPP
