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
      steering_(-0.25f),                 // 기본 스티어링 초기값
      throttle_(0.0f),                   // 기본 스로틀 초기값
      impl_(new Impl()),                 // Impl 구조체 동적 할당
      manual_mode_(true),                // 초기 모드를 수동으로 설정
      manual_throttle_(0.0f),            // 수동 입력용 스로틀
      manual_steering_(0.0f),            // 수동 입력용 스티어링
      gamepad_running_(false)            // 게임패드 스레드 실행 플래그 초기화
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
    py::gil_scoped_acquire acquire; // Python 호출 전 GIL 획득

    std::cout << "[제어 출력] 모드: " << (manual_mode_ ? "수동" : "자동") << " | 상태: ";

    try {
        if (manual_mode_) {
            // 수동 모드: 조이스틱 입력값 그대로 적용
            throttle_ = manual_throttle_ - 0.25f;
            steering_ = manual_steering_;
        } else {
            // 자동 모드: 상태 머신 기반 제어
            if (drive_state_ == DriveState::DRIVE && !crosswalk_flag) {
                // 일반 주행 중
                if (crosswalk) {
                    // 횡단보도 감지 시 대기 상태로 전환
                    crosswalk_flag = true;
                    drive_state_ = DriveState::WAIT_AFTER_CROSSWALK;
                    wait_start_time_ = steady_clock::now();
                    std::cout << "[INFO] 횡단보도 감지됨 → 대기 시작\n";
                } else if (start_line) {
                    // 출발선 감지 시 정지 상태로 전환
                    drive_state_ = DriveState::STOP_AT_START_LINE;
                    std::cout << "[INFO] 출발선 감지 → 정지\n";
                }
            }
            else if (drive_state_ == DriveState::DRIVE && crosswalk_flag) {
                // 횡단보도 이후 주행 재개 및 정지선 처리
                if (crosswalk_ignore_stopline) {
                    // 무시 기간 이후 정지선 감지 재활성화
                    auto since_resume = duration_cast<seconds>(steady_clock::now() - crosswalk_resume_time).count();
                    if (since_resume > 2) {
                        crosswalk_ignore_stopline = false;
                        std::cout << "[INFO] 정지선 감지 다시 활성화됨\n";
                    }
                } else if (stop_line) {
                    // 정지선 감지 시 노란 차선 주행 전환
                    drive_state_ = DriveState::YELLOW_LINE_DRIVE;
                    std::cout << "[INFO] 정지선 감지됨 → 노란 차선 주행으로 전환\n";
                }
            }
            else if (drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                // 대기 후 지정 시간 경과 시 주행 재개
                auto elapsed = duration_cast<seconds>(steady_clock::now() - wait_start_time_).count();
                if (elapsed >= WAIT_SECONDS) {
                    crosswalk_ignore_stopline = true;  // 정지선 무시 시작
                    crosswalk_resume_time = steady_clock::now();
                    drive_state_ = DriveState::DRIVE;
                    std::cout << "[INFO] 횡단보도 정지 후 주행 재개\n";
                }
            }
            else if (drive_state_ == DriveState::STOP_AT_START_LINE) {
                // 정지 상태: throttle_ = 0 로 설정됨
            }
            else if (drive_state_ == DriveState::YELLOW_LINE_DRIVE) {
                // 노란 차선 주행 상태: 좌측 ROI 제거, 흰색 주행 비활성화
                ROI_REMOVE_LEFT = true;
                WHITE_LINE_DRIVE = false;
                // 노란 픽셀 감소 시 일반 주행으로 복귀
                if (yellow_pixel_count < YELLOW_PIXEL_THRESHOLD) {
                    drive_state_ = DriveState::DRIVE;
                    ROI_REMOVE_LEFT = false;
                    WHITE_LINE_DRIVE = true;
                    std::cout << "[INFO] 노란 차선 사라짐 → 일반 흰색 차선 주행으로 전환\n";
                }
            }

            // 스로틀 설정: 정지 상태면 0, 아니면 함수 호출
            if (drive_state_ == DriveState::STOP_AT_START_LINE ||
                drive_state_ == DriveState::WAIT_AFTER_CROSSWALK) {
                throttle_ = 0.0f;
            } else {
                throttle_ = computeThrottle(cross_offset);
            }
            // 스티어링 설정: 차선 오프셋 기반 계산
            steering_ = computeSteering(cross_offset);
        }
        // PiRacerPro Python 객체에 제어 명령 전송
        impl_->piracer_.attr("set_steering_percent")(steering_);
        impl_->piracer_.attr("set_throttle_percent")(throttle_);
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 제어 실패: " << e.what() << "\n";
    }
    // 현재 상태 로그 출력
    switch (drive_state_) {
        case DriveState::DRIVE:                  std::cout << "주행"; break;
        case DriveState::WAIT_AFTER_CROSSWALK:   std::cout << "횡단보도 대기"; break;
        case DriveState::STOP_AT_START_LINE:     std::cout << "출발선 정지"; break;
        default:                                 break;
    }
    std::cout << " | cross_offset: " << cross_offset
              << " | steering: " << steering_
              << " | throttle: " << throttle_ << "\n";
}

// computeSteering: 오프셋 기반 조향 계산 (비례 제어 + 범위 제한)
float Controller::computeSteering(int offset) const {
    return std::clamp(-0.25f + STEERING_KP * offset, -0.7f, 0.7f);
}

// computeThrottle: 현재 고정 스로틀 반환 (추후 속도 제어 로직 보완 가능)
float Controller::computeThrottle(int /*offset*/) const {
    return BASE_THROTTLE;
}
