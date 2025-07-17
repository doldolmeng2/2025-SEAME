#pragma once
#include <chrono>
#include "constants.hpp"

// (1) enum class DrivePhase는 클래스 바깥에!
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

    void update(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount);
    void setManualMode(bool manual);
    float getSteering() const { return steering_; }
    float getThrottle() const { return throttle_; }

private:
    float computeSteering(int error);
    float computeThrottle(int error);
    float computeThrottle(int error, float baseThrottle);
    void handleAutomatic(bool stopLine, bool crosswalk, bool startLine, int laneOffset, int yellowCount);
    void transitionTo(DrivePhase newPhase);
    void setMode(Constants::FollowLaneMode mode);

    DrivePhase phase_;
    std::chrono::steady_clock::time_point phaseStartTime_;
    bool firstStopPassed_;
    bool manualMode_;

    float prevError_;
    float integral_;
    float steering_;
    float throttle_;
    float kp_, ki_, kd_, throttle_kp_;
    float baseSteering_;
    float maxError_;
};
