// control.hpp
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <chrono>
#include <vector>
#include <pybind11/embed.h>  // pybind11 임포트 필요

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

    pybind11::object piracer_;  // ← Python 객체를 위한 멤버 추가
};

#endif
