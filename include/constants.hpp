#pragma once
#include <string>

// 전역 변수 선언
extern int FRAME_WIDTH;
extern int FRAME_HEIGHT;
extern int WHITE_S_MAX;
extern int WHITE_V_MIN;
extern int VALID_V_MIN;
extern int YELLOW_H_MIN;
extern int YELLOW_H_MAX;
extern bool VIEWER;
extern float STEERING_KP;
extern float THROTTLE_KP;
extern float MAX_THROTTLE;
extern float BASE_THROTTLE;
extern int Y_TOP;
extern int LONG_HALF;
extern int SHORT_HALF;
extern int DEFAULT_LANE_GAP;
extern float AVG_PARAM;
extern float INTER_PARAM;
extern float STEERING_OFFSET;
extern bool ROI_REMOVE_LEFT;
extern float ROI_REMOVE_LEFT_X_THRESHOLD;

// 초기화 함수 선언
void load_constants(const std::string& path = "../constants.json");
