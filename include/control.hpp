// control.hpp
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <thread>
#include <atomic>
#include <chrono>

enum class DriveState {
    DRIVE
};

class Controller {
public:
    Controller();
    ~Controller();

    void update(int cross_offset);

private:
    // ── 기존 멤버 ──
    DriveState drive_state_;
    float steering_;
    float throttle_;

    float computeSteering(int offset) const;
    float computeThrottle(int offset) const;
};

#endif // CONTROL_HPP
