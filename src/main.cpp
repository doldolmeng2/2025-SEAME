// main.cpp
#include <iostream> // 표준 입출력 스트림
#include <thread> // 멀티스레드 구현을 위한 쓰레드 라이브러리
#include <mutex> // 상호 배제(뮤텍스)
#include <condition_variable> // 조건 변수
#include <atomic> // 원자성 변수
#include <memory> // 스마트 포인터
#include <string> // 문자열 처리
#include <csignal> // 시그널 처리
#include <chrono> // 시간 측정 및 sleep
#include <ctime> // 시간 변환
#include <iomanip> // 입출력 포맷 조정
#include <sstream> // 문자열 스트림 처리

#include "usb_cam.hpp" // USB 카메라 래퍼 클래스
#include "video_recorder.hpp" // 비디오 녹화 클래스
#include "lane_detector.hpp" // 차선 검출 클래스
#include "object_detector.hpp" // 객체 검출 클래스
#include "control.hpp" // 조향 제어 클래스
#include "constants.hpp" // 상수 정의 및 로드

// 전역 변수 선언
static std::mutex frame_mutex; // 프레임 공유 시 동기화용 뮤텍스
static std::shared_ptr<cv::Mat> shared_frame = nullptr; // 최신 프레임 저장 포인터

static std::mutex lane_mutex; // 차선 오프셋 동기화용 뮤텍스
static std::atomic<int> mean_center_offset{0}; // 차선 중심 오프셋 (원자 변수)
std::atomic<int> yellow_pixel_count{0};  // lane_detector의 결과를 공유

static std::mutex object_mutex; // 객체 검출 플래그 동기화용 뮤텍스
static std::vector<bool> detections_flags(3, false); // 객체 검출 결과 플래그 (stop, cross, start)

static std::condition_variable control_cv; // 제어 스레드 알림용 조건 변수
static std::mutex control_mutex; // 제어 조건 변수용 뮤텍스
static bool control_ready = false; // 제어 가능 상태 플래그

static std::condition_variable first_frame_cv; // 첫 번째 프레임 대기용 조건 변수
static bool first_frame_ready = false; // 첫 번째 프레임 수신 여부
static std::atomic<bool> running{true}; // 프로그램 실행 상태 플래그

// 실행 모드 열거형
// - DRIVE       : 차선 및 객체 검출 후 주행 제어만 수행 (녹화하지 않음)
// - RECORD      : 카메라 영상을 파일로 녹화만 수행 (주행 제어하지 않음)
// - DRIVE_RECORD: 주행 제어와 영상 녹화를 동시에 수행
enum class Mode { DRIVE, RECORD, DRIVE_RECORD };
static Mode current_mode = Mode::DRIVE; // 기본 실행 모드는 DRIVE

// SIGINT 시그널(CTRL+C) 처리 함수
void signal_handler(int) {
    running = false; // 프로그램 종료 플래그 설정
    std::cout << "\n[INFO] 종료 시그널 감지됨. 프로그램 종료 중...\n";
}

// 날짜/시간 기반 파일명 생성 함수
std::string getTimestampedFilename(const std::string& base_dir) {
    auto now = std::chrono::system_clock::now(); // 현재 시간
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&t); // 로컬 시간 변환

    std::ostringstream oss;
    oss << base_dir << "/output_"
        << std::setw(2) << std::setfill('0') << tm->tm_mday  // 일
        << std::setw(2) << std::setfill('0') << tm->tm_hour  // 시
        << std::setw(2) << std::setfill('0') << tm->tm_min   // 분
        << std::setw(2) << std::setfill('0') << tm->tm_sec   // 초
        << ".avi";
    return oss.str(); // 완성된 파일명 반환
}

int main(int argc, char** argv) {
    // 상수 파일 로드
    try {
        load_constants("constants.json"); // constants.json -> constants.hpp
        std::cout << "Steering Gain: " << STEERING_KP << "\n"; // 로드된 상수 출력
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 상수 로드 실패: " << e.what() << std::endl;
        return 1;
    }

    signal(SIGINT, signal_handler); // SIGINT 시그널 핸들러 등록

    // 실행 모드 파싱 (d, r, dr)
    if (argc < 2) {
        std::cerr << "[ERROR] 실행 인자를 지정해주세요: d, r, dr 중 하나\n";
        return 1;
    }
    std::string mode_arg = argv[1]; // 명령줄 인자
    if (mode_arg == "d") {
        current_mode = Mode::DRIVE;
    } else if (mode_arg == "r") {
        current_mode = Mode::RECORD;
    } else if (mode_arg == "dr") {
        current_mode = Mode::DRIVE_RECORD;
    } else {
        std::cerr << "[ERROR] 잘못된 모드입니다. d, r, dr 중 하나를 선택해주세요.\n";
        return 1;
    }
    std::cout << "[INFO] 선택된 모드: " << mode_arg << "\n";

    // 카메라 초기화
    USBCam cam;
    if (!cam.init()) {
        std::cerr << "[ERROR] 카메라 초기화 실패\n";
        return 1;
    }

    // 비디오 녹화 초기화 (레코드 또는 DRIVE_RECORD 모드)
    VideoRecorder recorder;
    if (current_mode == Mode::RECORD || current_mode == Mode::DRIVE_RECORD) {
        std::string filename = getTimestampedFilename("/home/orda/records/avis");
        if (!recorder.init(filename, FRAME_WIDTH, FRAME_HEIGHT, 30.0)) {
            std::cerr << "[ERROR] 비디오 저장 초기화 실패\n";
            return 1;
        }
    }

    // 카메라 캡처 스레드 (모든 모드에서 실행)
    std::thread camera_thread([&]() {
        while (running.load()) {
            cv::Mat frame = cam.getFrame(); // 프레임 읽기
            if (frame.empty()) continue; // 유효 프레임 아니면 스킵

            // 최신 프레임 공유
            auto ptr = std::make_shared<cv::Mat>(frame);
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                shared_frame = ptr;
                if (!first_frame_ready) {
                    first_frame_ready = true;
                    first_frame_cv.notify_all(); // 첫 프레임 수신 알림
                }
            }

            // RECORD, DRIVE_RECORD 모드에서만 녹화 수행
            if (current_mode == Mode::RECORD || current_mode == Mode::DRIVE_RECORD) {
                recorder.write(frame); // 녹화
            }

            // VIEWER 모드 화면 출력 및 ESC키 종료
            if (VIEWER) {
                // cv::imshow("Live", frame);
                if (cv::waitKey(1) == 27) {
                    running = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // CPU 과부하 방지
        }
    });

    // 첫 번째 프레임 수신 대기
    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        first_frame_cv.wait(lock, [] { return first_frame_ready; });
    }

    // DRIVE, DRIVE_RECORD 모드에서만 실행할 스레드
    std::thread lane_thread;
    std::thread object_thread;
    std::thread control_thread;
    if (current_mode == Mode::DRIVE || current_mode == Mode::DRIVE_RECORD) {
        // 차선 검출 스레드
        lane_thread = std::thread([&]() {
            LaneDetector lanedetector;
            while (running.load()) {
                std::shared_ptr<cv::Mat> frame;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    frame = shared_frame;
                }
                if (frame && !frame->empty()) {
                    cv::Mat vis_out;
                    int offset = lanedetector.process(*frame, vis_out); // 차선 오프셋 계산
                    yellow_pixel_count = lanedetector.getYellowPixelCount();
                    {
                        std::lock_guard<std::mutex> lock(lane_mutex);
                        mean_center_offset = offset; // 전역 오프셋 갱신
                    }
                    {
                        std::lock_guard<std::mutex> lock(control_mutex);
                        control_ready = true;
                        control_cv.notify_one(); // 제어 스레드 실행 알림
                    }
                    if (VIEWER) {
                        cv::imshow("Lane", vis_out);
                        if (cv::waitKey(1) == 27) running = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // 객체 검출 스레드
        object_thread = std::thread([&]() {
            ObjectDetector detector;
            while (running.load()) {
                std::shared_ptr<cv::Mat> frame;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    frame = shared_frame;
                }
                if (frame && !frame->empty()) {
                    cv::Mat vis_out;
                    std::vector<bool> flags;
                    detector.process(*frame, vis_out, flags); // 객체 검출
                    {
                        std::lock_guard<std::mutex> lock(object_mutex);
                        detections_flags = flags; // 검출 결과 저장
                    }
                    {
                        std::lock_guard<std::mutex> lock(control_mutex);
                        control_ready = true;
                        control_cv.notify_one(); // 제어 스레드 실행 알림
                    }
                    if (VIEWER) {
                        cv::imshow("Objects", vis_out);
                        if (cv::waitKey(1) == 27) running = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // 조향 제어 스레드
        control_thread = std::thread([&]() {
            Controller controller;
            while (running.load()) {
                std::unique_lock<std::mutex> lock(control_mutex);
                control_cv.wait(lock, [] { return control_ready; }); // 알림 대기
                control_ready = false;
                lock.unlock();

                // 최근 검출 결과 가져오기
                bool stop = false, cross = false, start = false;
                int offset = 0;
                {
                    std::lock_guard<std::mutex> lock(lane_mutex);
                    offset = mean_center_offset;
                }
                {
                    std::lock_guard<std::mutex> lock(object_mutex);
                    if (detections_flags.size() > 0) stop = detections_flags[0];
                    if (detections_flags.size() > 1) cross = detections_flags[1];
                    if (detections_flags.size() > 2) start = detections_flags[2];
                }
                int yellow_count = yellow_pixel_count.load();
		            controller.update(stop, cross, start, offset, yellow_count);
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 제어 주기 조절
            }
        });
    }

    // 스레드 종료 대기
    camera_thread.join();
    if (lane_thread.joinable()) lane_thread.join();
    if (object_thread.joinable()) object_thread.join();
    if (control_thread.joinable()) control_thread.join();

    // 비디오 녹화 자원 해제
    recorder.release();

    std::cout << "[INFO] 프로그램 종료\n";
    return 0;
}
