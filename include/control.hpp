// control.hpp
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <thread>
#include <atomic>
#include <chrono>
#include <pybind11/embed.h>

enum class DriveState {
    DRIVE,
    WAIT_AFTER_CROSSWALK,
    STOP_AT_START_LINE,
    YELLOW_LINE_DRIVE
};

class Controller {
public:
    Controller();
    ~Controller();

    void update(bool stop_line, bool crosswalk, bool start_line, int cross_offset, int yellow_pixel_count);

private:
    // ── 기존 멤버 ──
    DriveState drive_state_;
    std::chrono::steady_clock::time_point wait_start_time_;
    float steering_;
    float throttle_;

    float computeSteering(int offset) const;
    float computeThrottle(int offset) const;

    struct Impl;
    Impl* impl_;

    // ── 추가할 멤버 ──
    std::atomic<bool>   manual_mode_{false};
    std::atomic<float>  manual_throttle_{0.0f};
    std::atomic<float>  manual_steering_{0.0f};

    std::atomic<bool>   gamepad_running_{false};
    std::thread         gamepad_thread_;
    void startGamepadThread();
};

#endif // CONTROL_HPP
