#pragma once
#include <string>

// 전역 변수 선언
extern int FRAME_WIDTH;
extern int FRAME_HEIGHT;
extern int ROI_Y_START;
extern int ROI_Y_END;
extern int WHITE_S_MAX;
extern int WHITE_V_MIN;
extern int VALID_V_MIN;
extern int YELLOW_H_MIN;
extern int YELLOW_H_MAX;
extern bool VIEWER;
extern float STEERING_KP;
extern float THROTTLE_KP;
extern float MAX_THROTTLE;
extern int WAIT_SECONDS;

// 초기화 함수 선언
void load_constants(const std::string& path = "../constants.json");
