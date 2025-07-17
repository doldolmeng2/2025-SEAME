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
extern float BASE_THROTTLE;
extern int WAIT_SECONDS;
extern int RED_H_MIN1;
extern int RED_H_MIN2;
extern int RED_H_MAX1;
extern int RED_H_MAX2;
extern int RED_S_MIN;
extern int RED_V_MIN;
extern int Y_TOP;
extern int LONG_HALF;
extern int SHORT_HALF;
extern bool ROI_REMOVE_LEFT;
extern int ROI_REMOVE_LEFT_X_THRESHOLD;
extern bool WHITE_LINE_DRIVE;
extern int YELLOW_PIXEL_THRESHOLD;
extern int DEFAULT_LANE_GAP;
extern float STOPLINE_DETECTION_Y1;
extern float STOPLINE_DETECTION_Y2;

// 초기화 함수 선언
void load_constants(const std::string& path = "../constants.json");
