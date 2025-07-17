// control.cpp
#include "control.hpp"
#include "constants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include <pybind11/embed.h>

namespace py = pybind11;  // pybind11 네임스페이스 별칭
using namespace std::chrono;  // 시간 관련 유틸 사용

// 전역 상태 변수들
bool crosswalk_flag = false;               // 횡단보도 감지 플래그 (한 번만 처리)
bool crosswalk_ignore_stopline = false;    // 횡단보도 후 정지선 무시 여부
steady_clock::time_point crosswalk_resume_time;  // 정지선 무시 기간 시작 시간 저장

// Python 객체(piracer, gamepad)를 감싸는 내부 구현 구조체
struct __attribute__((visibility("hidden"))) Controller::Impl {
    py::object piracer_;   // PiRacerPro Python 객체
    py::object gamepad_;   // ShanWanGamepad Python 객체
};

// 생성자: Python 인터프리터 초기화 및 객체 생성, 게임패드 스레드 시작
Controller::Controller()
    : drive_state_(DriveState::DRIVE),   // 초기 주행 상태 설정
      steering_(-0.35f),                 // 기본 스티어링 초기값
      throttle_(0.0f),                   // 기본 스로틀 초기값
      impl_(new Impl()),                 // Impl 구조체 동적 할당
      manual_mode_(true),                // 초기 모드를 수동으로 설정
      manual_throttle_(0.0f),            // 수동 입력용 스로틀
      manual_steering_(0.0f),            // 수동 입력용 스티어링
      gamepad_running_(false),           // 게임패드 스레드 실행 플래그 초기화
      last_manual_mode_(true)            // 자동 -> 수동 변경시 리셋

{
    try {
        // Python 인터프리터 시작
        py::initialize_interpreter();

        // PiRacerPro 객체 생성 (파이썬 모듈 piracer.vehicles에서 가져옴)
        auto piracer_module = py::module_::import("piracer.vehicles");
        impl_->piracer_ = piracer_module.attr("PiRacerPro")();

        // ShanWanGamepad 객체 생성 (파이썬 모듈 piracer.gamepads에서 가져옴)
        auto gamepad_module = py::module_::import("piracer.gamepads");
        impl_->gamepad_ = gamepad_module.attr("ShanWanGamepad")();

        std::cout << "[INFO] Python PiracerPro 및 ShanWanGamepad 생성 완료\n";

        // 별도 스레드에서 게임패드 입력 처리 시작
        startGamepadThread();
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 초기화 실패: " << e.what() << "\n";
    }
}

// 소멸자: 게임패드 스레드 종료, 모터 정지, Python 인터프리터 종료
Controller::~Controller() {
    // 게임패드 스레드 종료 요청
    gamepad_running_ = false;
    if (gamepad_thread_.joinable()) {
        gamepad_thread_.join();  // 스레드가 끝날 때까지 대기
    }

    // 모터를 완전히 중지시켜 안전 확보
    try {
        if (impl_ && impl_->piracer_) {
            impl_->piracer_.attr("set_throttle_percent")(0.0f);
            impl_->piracer_.attr("set_steering_percent")(0.0f);
            std::cout << "[INFO] 종료 전 모터 정지 명령 전송\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 종료 시 모터 정지 실패: " << e.what() << "\n";
    }

    delete impl_;               // Impl 메모리 해제
    py::finalize_interpreter(); // Python 인터프리터 종료
}

//게임패드 입력 전용 스레드 시작 함수
void Controller::startGamepadThread() {
    gamepad_running_ = true;
    gamepad_thread_ = std::thread([this]() {
        while (gamepad_running_) {
            try {
                // Python GIL 획득하여 안전하게 호출
                py::gil_scoped_acquire gil;

                // gamepad.read_data()를 통해 입력 데이터 가져오기
                auto data = impl_->gamepad_.attr("read_data")();

                // A 버튼 누르면 수동 모드, B 버튼 누르면 자동 모드 전환
                if (py::bool_(data.attr("button_a"))) manual_mode_ = true;
                if (py::bool_(data.attr("button_b"))) manual_mode_ = false;

                if (last_manual_mode_ != manual_mode_) {
                    std::cout << "[INFO] 모드 전환 감지됨: "
                    << (manual_mode_ ? "수동 모드로 전환" : "자동 모드로 전환\n") << std::endl;
                    last_manual_mode_ = manual_mode_;

                    //수동 모드로 전환될 때 상태 초기화
                    if (manual_mode_) {
                        drive_state_ = DriveState::DRIVE;
                        crosswalk_flag = false;
                        crosswalk_ignore_stopline = false;
                        ROI_REMOVE_LEFT = false;
                        WHITE_LINE_DRIVE = true;
                        std::cout << "[INFO] 수동 모드 진입 -> 내부 상태 초기화 완료\n";
                    }
                }

                // 우측 스틱 Y축 -> throttle, 좌측 스틱 X축 -> steering
                manual_throttle_ = data.attr("analog_stick_right").attr("y").cast<float>() * 0.5f;
                manual_steering_ = data.attr("analog_stick_left").attr("x").cast<float>();
            }
            catch (const std::exception& e) {
                std::cerr << "[WARN] Gamepad read failed: " << e.what() << "\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 짧게 대기 후 재시도
            }
        }
    });
}

// update: 메인 루프에서 호출되어 주행 제어 로직 수행
// stop_line: 정지선 감지 여부
// crosswalk: 횡단보도 감지 여부
// start_line: 출발선 감지 여부
// cross_offset: 차선 중심 대비 오프셋
// yellow_pixel_count: 노란색 차선 픽셀 수
void Controller::update(bool stop_line, bool crosswalk, bool start_line, int cross_offset, int yellow_pixel_count) {
    py::gil_scoped_acquire acquire;

    try {
        if (manual_mode_) {
            throttle_ = manual_throttle_ - 0.2f;
            steering_ = manual_steering_ - 0.35f;
        } else {
            // 자동 모드: 상태 머신 기반 제어
            if (drive_state_ == DriveState::DRIVE && !is_waiting_for_crosswalk) {
                if (crosswalk) {
                    // 횡단보도 감지 시 대기 상태로 전환
                    is_waiting_for_crosswalk = true;
                    drive_state_ = DriveState::WAIT_AFTER_CROSSWALK;
                    std::cout << "[INFO] 횡단보도 감지됨 → 대기 시작\n";
                }
            } 
            else if (drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                // 횡단보도 대기 후
                drive_state_ = DriveState::DRIVE;
                std::cout << "[INFO] 횡단보도 대기 종료\n";
            }

            // 정지선 처리
            if (stop_line) {
                // 정지선 감지 시
                follow_right_lane = false;  // 오른쪽 차선에서 왼쪽 차선으로 전환
                follow_left_lane = true;  // 왼쪽 차선 추적
                throttle_ = 0.0f;  // 정지
                steering_ = 0.0f;  // 정지
                std::cout << "[INFO] 정지선 감지됨 → 왼쪽 차선으로 전환\n";
            } else {
                throttle_ = computeThrottle(cross_offset);  // P 제어를 통한 스로틀 값 계산
                steering_ = computeSteering(cross_offset);  // PID 제어를 통한 조향값 계산
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 제어 실패: " << e.what() << "\n";
    }
}


float Controller::computeSteering(int offset) {
    // P, I, D를 위한 변수들
    float p_term = STEERING_KP * offset;  // 비례
    integral += offset;  // 적분
    float i_term = STEERING_KI * integral;  // 적분
    float d_term = STEERING_KD * (offset - prev_error);  // 미분

    // PID 제어 계산
    float steering = p_term + i_term + d_term;

    // 이전 오차를 현재 오차로 갱신
    prev_error = offset;

    // 조향 범위 제한
    return std::clamp(steering, -1.0f, 1.0f);
}

// 스로틀 계산을 위한 P 제어
float Controller::computeThrottle(int offset) {
    // 차선의 오프셋에 따른 스로틀 값 계산 (P 제어)
    float throttle = THROTTLE_KP * offset;  // 차선 오프셋에 비례하는 스로틀 값

    // 최소/최대 속도 제한
    return std::clamp(throttle, -MAX_THROTTLE, MAX_THROTTLE);
}
