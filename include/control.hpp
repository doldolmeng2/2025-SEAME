// control.hpp
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <thread>
#include <atomic>
#include <chrono>
#include <pybind11/embed.h>
#include "constants.hpp"

class Controller {
public:
    Controller();
    ~Controller();

    void update(bool stop_line, bool crosswalk, bool start_line, int cross_offset, int yellow_pixel_count);

private:
    DriveState drive_state_; // enum DriveState의 원소 drive_state_
    
    bool crosswalk_flag = false;               // 횡단보도 감지 플래그 (한 번만 처리)
    bool crosswalk_ignore_stopline = false;    // 횡단보도 후 정지선 무시 여부
    std::chrono::steady_clock::time_point crosswalk_resume_time;  // 정지선 무시 기간 시작 시간 저장
    std::chrono::steady_clock::time_point yellow_mode_start_time_;
    std::chrono::steady_clock::time_point wait_start_time_; // 횡단보도 감지된 시점
    
    float steering_;
    float throttle_;
    bool last_manual_mode_;

    float computeSteering(int offset) const;
    float computeThrottle(int offset) const;

    struct Impl;
    Impl* impl_;

    std::atomic<bool>   manual_mode_{false};
    std::atomic<float>  manual_throttle_{0.0f};
    std::atomic<float>  manual_steering_{0.0f};

    std::atomic<bool>   gamepad_running_{false};
    std::thread         gamepad_thread_;
    void startGamepadThread();
};

#endif // CONTROL_HPP
