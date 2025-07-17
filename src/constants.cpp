#include "constants.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

// 전역 변수 정의
int FRAME_WIDTH;
int FRAME_HEIGHT;
int ROI_Y_START;
int ROI_Y_END;
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
int WAIT_SECONDS;
int RED_H_MIN1;
int RED_H_MIN2;
int RED_H_MAX1;
int RED_H_MAX2;
int RED_S_MIN;
int RED_V_MIN;
int Y_TOP;
int LONG_HALF;
int SHORT_HALF;
bool ROI_REMOVE_LEFT;
int ROI_REMOVE_LEFT_X_THRESHOLD;
bool WHITE_LINE_DRIVE;
int YELLOW_PIXEL_THRESHOLD;
int DEFAULT_LANE_GAP;
float STOPLINE_DETECTION_Y1;
float STOPLINE_DETECTION_Y2;
float STOPLINE_DETECTION_THRESHOLD;

void load_constants(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("constants.json 파일 열기 실패");
    }

    nlohmann::json j;
    file >> j;

    FRAME_WIDTH = j["FRAME_WIDTH"];
    FRAME_HEIGHT = j["FRAME_HEIGHT"];
    ROI_Y_START = j["ROI_Y_START"];
    ROI_Y_END = j["ROI_Y_END"];
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
    WAIT_SECONDS = j["WAIT_SECONDS"];
    RED_H_MIN1= j["RED_H_MIN1"];
    RED_H_MIN2= j["RED_H_MIN2"];
    RED_H_MAX1= j["RED_H_MAX1"];
    RED_H_MAX2= j["RED_H_MAX2"];
    RED_S_MIN= j["RED_S_MIN"];
    RED_V_MIN= j["RED_V_MIN"];
    Y_TOP= j["Y_TOP"];
    LONG_HALF= j["LONG_HALF"];
    SHORT_HALF= j["SHORT_HALF"];
    ROI_REMOVE_LEFT= j["ROI_REMOVE_LEFT"];
    ROI_REMOVE_LEFT_X_THRESHOLD= j["ROI_REMOVE_LEFT_X_THRESHOLD"];
    WHITE_LINE_DRIVE = j["WHITE_LINE_DRIVE"];
    YELLOW_PIXEL_THRESHOLD = j["YELLOW_PIXEL_THRESHOLD"];
    DEFAULT_LANE_GAP = j["DEFAULT_LANE_GAP"];
    STOPLINE_DETECTION_Y1 = j["STOPLINE_DETECTION_Y1"];
    STOPLINE_DETECTION_Y2 = j["STOPLINE_DETECTION_Y2"];
    STOPLINE_DETECTION_THRESHOLD = j["STOPLINE_DETECTION_THRESHOLD"];
}
