#pragma once

#include <pybind11/embed.h>
#include <chrono>

namespace py = pybind11;

// 1. DrivePhase와 Constants를 올바르게 포함시킴
namespace Constants {
    const float BASE_THROTTLE = 0.5f;
    const float BASE_THROTTLE_LOW = 0.2f;
    const float BASE_THROTTLE_YELLOW = 0.3f;
    const float STEERING_KP = 1.0f;
    const float STEERING_KI = 0.1f;
    const float STEERING_KD = 0.01f;
    const float THROTTLE_KP = 0.1f;
    const float STEERING_OFFSET = 0.0f;
    const float DEFAULT_LANE_GAP = 20.0f;
    const int YELLOW_PIXEL_THRESHOLD = 100;
    const float MAX_THROTTLE = 1.0f;
    const bool WHITE_LINE_DRIVE = true;
    const int DEFAULT_LANE_GAP = 20;
    const float lane_follow_steering_bias = 0.05f;
}

enum class DrivePhase {
    START,
    CENTRE_WHITE,
    STOP_CROSSWALK,
    RIGHT_WHITE,
    CENTRE_YELLOW,
    LEFT_YELLOW,
    RIGHT_WHITE_AFTER,
    FINISH
};

class Controller {
public:
    Controller();
    ~Controller() = default;

    void setManualMode(bool manual);
    void update(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount);
    void handleAutomatic(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount);
    
    float getSteering() const { return steering_; }
    float getThrottle() const { return throttle_; }

private:
    // 2. Impl 구조체를 정의하여 Python 객체들을 감싸도록 한다.
    struct Impl {
        py::object piracer_;   // PiRacerPro Python 객체
        py::object gamepad_;   // ShanWanGamepad Python 객체
    };

    std::unique_ptr<Impl> impl_;  // Impl 객체 포인터

    // C++에서 주행을 제어하는 변수들
    float steering_;
    float throttle_;
    bool manualMode_;

    void transitionTo(DrivePhase newPhase);
    void setMode(Constants::FollowLaneMode mode);
    float computeSteering(int error);
    float computeThrottle(int error, float baseThrottle);
    float computeThrottle(int error);
    
    // 주행 상태 변수들
    DrivePhase phase_;
    std::chrono::steady_clock::time_point phaseStartTime_;
    bool firstStopPassed_;
    float prevError_;
    float integral_;
    float kp_, ki_, kd_, throttle_kp_;
    float baseSteering_;
    float maxError_;
};
