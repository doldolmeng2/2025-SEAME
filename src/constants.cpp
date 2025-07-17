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
int Y_TOP;
int LONG_HALF;
int SHORT_HALF;
bool ROI_REMOVE_LEFT;
int ROI_REMOVE_LEFT_X_THRESHOLD;
bool WHITE_LINE_DRIVE;
int YELLOW_PIXEL_THRESHOLD;
int NUM_DIVISIONS;
int TARGET_INTERSECTION_X;
float CURVE_RMSE_THRESHOLD;
float FORCE_STEER_RIGHT;
float CURVE_KP;
float STRAIGHT_KP;

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
    Y_TOP= j["Y_TOP"];
    LONG_HALF= j["LONG_HALF"];
    SHORT_HALF= j["SHORT_HALF"];
    ROI_REMOVE_LEFT= j["ROI_REMOVE_LEFT"];
    ROI_REMOVE_LEFT_X_THRESHOLD= j["ROI_REMOVE_LEFT_X_THRESHOLD"];
    WHITE_LINE_DRIVE = j["WHITE_LINE_DRIVE"];
    YELLOW_PIXEL_THRESHOLD = j["YELLOW_PIXEL_THRESHOLD"];
    NUM_DIVISIONS = j["NUM_DIVISIONS"];
    TARGET_INTERSECTION_X = j["TARGET_INTERSECTION_X"];
    CURVE_RMSE_THRESHOLD = j["CURVE_RMSE_THRESHOLD"];
    FORCE_STEER_RIGHT = j["FORCE_STEER_RIGHT"];
    CURVE_KP = j["CURVE_KP"];
    STRAIGHT_KP = j["STRAIGHT_KP"];

}
