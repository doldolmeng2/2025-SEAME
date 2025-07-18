#include "usb_cam.hpp"
#include "video_recorder.hpp"
#include "lane_detector.hpp"
#include "object_detector.hpp"
#include "control.hpp"
#include "constants.hpp"
#include <pybind11/embed.h>  // Python 인터프리터를 임베드하기 위한 헤더
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <chrono>

namespace py = pybind11;
using namespace std::chrono_literals;
using namespace Constants;

class AutoDriveSystem {
public:
    enum class Mode { DRIVE, RECORD, DRIVE_RECORD };  // 자율주행 모드 (주행, 기록, 주행+기록)
    Mode currentMode{Mode::DRIVE};  // 초기 모드는 주행 모드

    std::shared_ptr<cv::Mat> getSharedFrame() {
        std::lock_guard<std::mutex> lk(frameMutex);
        return sharedFrame;
    }

    AutoDriveSystem() {
        // 종료 시 시스템이 종료되도록 신호 처리
        signal(SIGINT, [](int){ running = false; });
    }

    // 시스템 초기화 함수
    bool init(int argc, char** argv) {
        static py::scoped_interpreter guard{};  // Python 인터프리터 실행

        try {
            loadConstants("constants.json");  // 상수 로드
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] 상수 로드 실패: " << e.what() << "\n";
            return false;
        }

        try {
            // Python에서 PiRacer 객체를 불러옴
            py::object vehicles_mod = py::module_::import("piracer.vehicles");
            piracer_ = vehicles_mod.attr("PiRacerPro")();
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] piracer import 실패: " << e.what() << "\n";
            return false;
        }

        // 모드 파싱
        if (!parseMode(argc, argv)) return false;
        if (!cam_.init()) return false;  // 카메라 초기화

        // 비디오 기록이 필요하면 초기화
        if (needsRecording()) {
            std::string fn = makeFilename("/home/orda/records/avis");
            if (!recorder_.init(fn, FRAME_WIDTH, FRAME_HEIGHT, 30.0))
                return false;
        }

        return true;
    }

    // 자율주행 시스템 실행
    void run() {
        startThreads();  // 스레드 시작
        joinThreads();   // 스레드 종료 대기
        recorder_.release();  // 비디오 녹화 종료
        std::cout << "[INFO] 시스템 종료\n";
    }

private:
    // 각 스레드를 시작하는 함수
    void startThreads() {
        cameraThread_ = std::thread(&AutoDriveSystem::cameraLoop, this);  // 카메라 캡처 스레드
        if (isDriveMode()) {
            laneThread_    = std::thread(&AutoDriveSystem::laneLoop, this);    // 차선 감지 스레드
            objectThread_  = std::thread(&AutoDriveSystem::objectLoop, this);  // 객체 감지 스레드
            controlThread_ = std::thread(&AutoDriveSystem::controlLoop, this); // 차량 제어 스레드
        }
    }

    // 각 스레드를 종료할 때까지 대기하는 함수
    void joinThreads() {
        cameraThread_.join();
        if (laneThread_.joinable()) laneThread_.join();
        if (objectThread_.joinable()) objectThread_.join();
        if (controlThread_.joinable()) controlThread_.join();
    }

    // 카메라 캡처 및 프레임 처리 스레드
    void cameraLoop() {
        while (running) {
            cv::Mat frame = cam_.getFrame();  // 카메라에서 프레임 캡처
            if (frame.empty()) {
                std::cerr << "[ERROR] 빈 프레임을 읽었습니다. 계속 시도합니다.\n";
                continue;  // 프레임이 비어 있으면 건너뛰기
            }
            {
                std::lock_guard lk(frameMutex);
                sharedFrame = std::make_shared<cv::Mat>(frame);
                if (!firstFrameArrived) {
                    firstFrameArrived = true;
                    firstFrameCv.notify_all();  // 첫 번째 프레임 도착 대기
                    std::cerr << "[INFO] 첫 번째 프레임 도착" << std::endl;
                }
            }

            if (needsRecording()) recorder_.write(frame);  // 녹화 모드일 경우 비디오 파일에 프레임 저장
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 차선 감지 스레드
    void laneLoop() {
        waitFirstFrame();  // 첫 번째 프레임이 도착할 때까지 대기
        while (running) {
            auto frame = getSharedFrame();  // 공유된 프레임 가져오기
            if (!frame) continue;  // 프레임이 없다면 건너뛰기

            cv::Mat vis;
            int offset = laneDetector_.process(*frame, vis);  // 차선 감지

            {
                std::lock_guard lk(laneMutex);
                laneOffset = offset;
                yellowCount = laneDetector_.getYellowPixelCount();
            }

            notifyControl();  // 제어 스레드에 알리기
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 객체 감지 스레드
    void objectLoop() {
        waitFirstFrame();  // 첫 번째 프레임이 도착할 때까지 대기
        while (running) {
            auto frame = getSharedFrame();  // 공유된 프레임 가져오기
            if (!frame) continue;  // 프레임이 없다면 건너뛰기

            cv::Mat vis;
            auto results = objectDetector_.detect(*frame, vis);  // 객체 감지

            std::vector<bool> flags = {
                results.stopline,
                results.crosswalk,
                results.startline
            };

            {
                std::lock_guard lk(objectMutex);
                objectFlags = flags;  // 객체 감지 결과 저장
            }

            notifyControl();  // 제어 스레드에 알리기
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 차량 제어 스레드
    void controlLoop() {
        Controller ctrl;
        waitFirstFrame();  // 첫 번째 프레임이 도달할 때까지 대기
        while (running) {
            std::unique_lock ul(controlMutex);
            controlCv.wait(ul, [this]{ return controlReady; });  // 제어 대기
            std::cerr << "[INFO] 제어 스레드 시작, controlReady: " << controlReady << std::endl;  // 디버깅 메시지

            controlReady = false;  // 제어 후에는 반드시 false로 리셋해야 함

            int offset, yellow;
            std::vector<bool> flags;
            {
                std::lock_guard lk1(laneMutex);
                offset = laneOffset;
                yellow = yellowCount;
            }
            {
                std::lock_guard lk2(objectMutex);
                flags = objectFlags;
            }
            ul.unlock();

            // 차량 제어 업데이트
            ctrl.update(flags[0], flags[1], flags[2], offset, yellow);  

            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 적절한 대기 시간
        }
    }

    // 첫 번째 프레임 도착 대기
    void waitFirstFrame() {
        std::unique_lock ul(frameMutex);
        firstFrameCv.wait(ul, [this]{ return firstFrameArrived; });
        std::cerr << "[INFO] 첫 번째 프레임 대기 종료" << std::endl;  // 추가된 디버깅 메시지
    }

    // 제어 스레드에 알리는 함수
    void notifyControl() {
        {
            std::lock_guard lk(controlMutex);
            controlReady = true;
        }
        controlCv.notify_one();
        std::cerr << "[INFO] 제어 스레드 알림" << std::endl;  // 디버깅 메시지
    }

    // 모드 파싱 함수 (주행 모드 설정)
    bool parseMode(int argc, char** argv) {
        if (argc < 2) {
            std::cerr<<"[ERROR] 모드 인자 필요 (d, r, dr)\n";
            return false;
        }
        std::string m{argv[1]};
        if      (m=="d")  currentMode = Mode::DRIVE;
        else if (m=="r")  currentMode = Mode::RECORD;
        else if (m=="dr") currentMode = Mode::DRIVE_RECORD;
        else {
            std::cerr<<"[ERROR] 잘못된 모드: "<<m<<"\n";
            return false;
        }
        std::cout<<"[INFO] 모드: "<<m<<"\n";
        return true;
    }

    // 주행 모드 여부 확인
    bool isDriveMode() const {
        return currentMode == Mode::DRIVE ||
               currentMode == Mode::DRIVE_RECORD;
    }

    // 녹화가 필요한지 확인
    bool needsRecording() const {
        return currentMode == Mode::RECORD ||
               currentMode == Mode::DRIVE_RECORD;
    }

    // 파일명 생성 함수
    std::string makeFilename(const std::string& base) {
        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        std::tm tm{}; localtime_r(&t, &tm);
        std::ostringstream ss;
        ss << base << "/out_"
           << std::setw(2) << std::setfill('0') << tm.tm_mday
           << std::setw(2) << tm.tm_hour
           << std::setw(2) << tm.tm_min
           << std::setw(2) << tm.tm_sec
           << ".avi";
        return ss.str();
    }

    // 멤버 변수들
    USBCam         cam_;           // 카메라 객체
    VideoRecorder  recorder_;      // 비디오 녹화기 객체
    LaneDetector   laneDetector_;  // 차선 감지기 객체
    ObjectDetector objectDetector_;// 객체 감지기 객체

    // 스레드 관련 변수들
    std::thread             cameraThread_, laneThread_, objectThread_, controlThread_;
    static inline std::atomic<bool> running{true};

    // 공유된 프레임을 위한 뮤텍스
    std::mutex                    frameMutex;
    std::condition_variable       firstFrameCv;
    bool                          firstFrameArrived{false};
    std::shared_ptr<cv::Mat>      sharedFrame;

    // 차선 감지 관련 변수들
    std::mutex                    laneMutex;
    int                           laneOffset{0};
    int                           yellowCount{0};

    // 객체 감지 관련 변수들
    std::mutex                    objectMutex;
    std::vector<bool>             objectFlags = { false, false, false };  // 객체 감지 플래그 초기화


    // 제어 관련 변수들
    std::mutex                    controlMutex;
    std::condition_variable       controlCv;
    bool                          controlReady{false};

    // 파이썬 PiRacer 객체
    py::object piracer_;
};

int main(int argc, char** argv) {
    AutoDriveSystem system;
    if (!system.init(argc, argv)) return 1;  // 초기화 실패 시 종료
    system.run();  // 시스템 실행
    return 0;
}
