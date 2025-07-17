// main.cpp
#include "usb_cam.hpp"
#include "video_recorder.hpp"
#include "lane_detector.hpp"
#include "object_detector.hpp"
#include "control.hpp"
#include "constants.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <pybind11/embed.h>

namespace py = pybind11;
using namespace std::chrono_literals;
using namespace Constants;

class AutoDriveSystem {
public:
    enum class Mode { DRIVE, RECORD, DRIVE_RECORD };
    Mode currentMode{Mode::DRIVE};

    AutoDriveSystem() {
        signal(SIGINT, [](int){ running = false; });
    }

    bool init(int argc, char** argv) {
        static py::scoped_interpreter guard{}; // 파이썬 인터프리터 실행
        try {
            loadConstants("constants.json");
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] 상수 로드 실패: " << e.what() << "\n";
            return false;
        }
        try {
            py::object piracer_mod = py::module_::import("piracer");
            piracer_ = piracer_mod.attr("PiRacerProPlatform")();
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] piracer import 실패: " << e.what() << "\n";
            return false;
        }
        if (!parseMode(argc, argv)) return false;
        if (!cam_.init())      return false;
        if (needsRecording()) {
            std::string fn = makeFilename("/home/orda/records/avis");
            if (!recorder_.init(fn, FRAME_WIDTH, FRAME_HEIGHT, 30.0))
                return false;
        }
        return true;
    }

    void run() {
        startThreads();
        joinThreads();
        recorder_.release();
        std::cout << "[INFO] 시스템 종료\n";
    }

private:
    void startThreads() {
        cameraThread_ = std::thread(&AutoDriveSystem::cameraLoop, this);
        if (isDriveMode()) {
            laneThread_    = std::thread(&AutoDriveSystem::laneLoop, this);
            objectThread_  = std::thread(&AutoDriveSystem::objectLoop, this);
            controlThread_ = std::thread(&AutoDriveSystem::controlLoop, this);
        }
    }

    void joinThreads() {
        cameraThread_.join();
        if (laneThread_.joinable())    laneThread_.join();
        if (objectThread_.joinable())  objectThread_.join();
        if (controlThread_.joinable()) controlThread_.join();
    }

    void cameraLoop() {
        while (running) {
            cv::Mat frame = cam_.getFrame();
            if (frame.empty()) continue;
            {
                std::lock_guard lk(frameMutex);
                sharedFrame = std::make_shared<cv::Mat>(frame);
                if (!firstFrameArrived) {
                    firstFrameArrived = true;
                    firstFrameCv.notify_all();
                }
            }
            if (needsRecording()) recorder_.write(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void laneLoop() {
        waitFirstFrame();
        while (running) {
            auto frame = getSharedFrame();
            if (!frame) continue;
            cv::Mat vis;
            int offset = laneDetector_.process(*frame, vis);
            {
                std::lock_guard lk(laneMutex);
                laneOffset = offset;
                yellowCount = laneDetector_.getYellowPixelCount();
            }
            notifyControl();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void objectLoop() {
        waitFirstFrame();
        while (running) {
            auto frame = getSharedFrame();
            if (!frame) continue;
            cv::Mat vis;
            auto results = objectDetector_.detect(*frame, vis);
            std::vector<bool> flags = {
                results.stopline,
                results.crosswalk,
                results.startline
            };
            {
                std::lock_guard lk(objectMutex);
                objectFlags = flags;
            }
            notifyControl();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void controlLoop() {
        Controller ctrl;
        waitFirstFrame();
        while (running) {
            std::unique_lock ul(controlMutex);
            controlCv.wait(ul, [this]{ return controlReady; });
            controlReady = false;

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

            ctrl.update(flags[0], flags[1], flags[2], offset, yellow);

            // ---- 여기서 파이썬 piracer에 명령! ----
            piracer_.attr("set_steering")(ctrl.getSteering());
            piracer_.attr("set_throttle")(ctrl.getThrottle());
            // -------------------------------------

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void waitFirstFrame() {
        std::unique_lock ul(frameMutex);
        firstFrameCv.wait(ul, [this]{ return firstFrameArrived; });
    }

    void notifyControl() {
        {
            std::lock_guard lk(controlMutex);
            controlReady = true;
        }
        controlCv.notify_one();
    }

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

    bool isDriveMode() const {
        return currentMode == Mode::DRIVE ||
               currentMode == Mode::DRIVE_RECORD;
    }
    bool needsRecording() const {
        return currentMode == Mode::RECORD ||
               currentMode == Mode::DRIVE_RECORD;
    }

    std::shared_ptr<cv::Mat> getSharedFrame() {
        std::lock_guard lk(frameMutex);
        return sharedFrame;
    }

    std::string makeFilename(const std::string& base) {
        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&t, &tm);
        std::ostringstream ss;
        ss << base << "/out_"
           << std::setw(2) << std::setfill('0') << tm.tm_mday
           << std::setw(2) << tm.tm_hour
           << std::setw(2) << tm.tm_min
           << std::setw(2) << tm.tm_sec
           << ".avi";
        return ss.str();
    }

    // ---- 멤버 변수 ----
    USBCam         cam_;
    VideoRecorder  recorder_;
    LaneDetector   laneDetector_;
    ObjectDetector objectDetector_;

    std::thread             cameraThread_, laneThread_, objectThread_, controlThread_;
    static inline std::atomic<bool> running{true};

    std::mutex                    frameMutex;
    std::condition_variable       firstFrameCv;
    bool                          firstFrameArrived{false};
    std::shared_ptr<cv::Mat>      sharedFrame;

    std::mutex                    laneMutex;
    int                           laneOffset{0};
    int                           yellowCount{0};

    std::mutex                    objectMutex;
    std::vector<bool>             objectFlags = std::vector<bool>(3, false);

    std::mutex                    controlMutex;
    std::condition_variable       controlCv;
    bool                          controlReady{false};

    // ---- 파이썬 piracer 객체 ----
    py::object piracer_;
};

int main(int argc, char** argv) {
    AutoDriveSystem system;
    if (!system.init(argc, argv)) return 1;
    system.run();
    return 0;
}
