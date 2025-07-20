#include "constants.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

// 전역 변수 정의
int FRAME_WIDTH;
int FRAME_HEIGHT;
int WHITE_S_MAX;
int WHITE_V_MIN;
int VALID_V_MIN;
int YELLOW_H_MIN;
int YELLOW_H_MAX;
bool VIEWER;
float STEERING_KP;
float THROTTLE_KP;
float MAX_THROTTLE;
float BASE_THROTTLE;
int Y_TOP;
int LONG_HALF;
int SHORT_HALF;
int DEFAULT_LANE_GAP;
float AVG_PARAM;
float INTER_PARAM;
float STEERING_OFFSET;
bool ROI_REMOVE_LEFT;
float ROI_REMOVE_LEFT_X_THRESHOLD;

void load_constants(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("constants.json 파일 열기 실패");
    }

    nlohmann::json j;
    file >> j;

    FRAME_WIDTH = j["FRAME_WIDTH"];
    FRAME_HEIGHT = j["FRAME_HEIGHT"];
    WHITE_S_MAX = j["WHITE_S_MAX"];
    WHITE_V_MIN = j["WHITE_V_MIN"];
    VALID_V_MIN = j["VALID_V_MIN"];
    YELLOW_H_MIN = j["YELLOW_H_MIN"];
    YELLOW_H_MAX = j["YELLOW_H_MAX"];
    VIEWER = j["VIEWER"];
    STEERING_KP = j["STEERING_KP"];
    THROTTLE_KP = j["THROTTLE_KP"];
    MAX_THROTTLE = j["MAX_THROTTLE"];
    BASE_THROTTLE = j["BASE_THROTTLE"];
    Y_TOP= j["Y_TOP"];
    LONG_HALF= j["LONG_HALF"];
    SHORT_HALF= j["SHORT_HALF"];
    DEFAULT_LANE_GAP = j["DEFAULT_LANE_GAP"];
    AVG_PARAM = j["AVG_PARAM"];
    INTER_PARAM = j["INTER_PARAM"];
    STEERING_OFFSET = j["STEERING_OFFSET"];
    ROI_REMOVE_LEFT = j["ROI_REMOVE_LEFT"];
    ROI_REMOVE_LEFT_X_THRESHOLD = j["ROI_REMOVE_LEFT_X_THRESHOLD"];
}
