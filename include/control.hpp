// control.hpp
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <chrono>
#include <vector>

enum class DriveState {
    DRIVE,
    WAIT_AFTER_CROSSWALK,
    STOP_AT_START_LINE
};

class Controller {
public:
    Controller();
    ~Controller();

    void update(bool stop_line, bool crosswalk, bool start_line, int cross_offset);
    float getSteering() const;
    float getThrottle() const;

private:
    float computeSteering(int offset) const;
    float computeThrottle(int offset) const;

    DriveState drive_state_;
    std::chrono::steady_clock::time_point wait_start_time_;
    float steering_;
    float throttle_;

    // pybind11 관련 멤버는 cpp 파일에서만 정의
    struct Impl;
    Impl* impl_;
};

#endif
