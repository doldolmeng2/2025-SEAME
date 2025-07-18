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
extern float STOPLINE_DETECTION_THRESHOLD;
extern float AVG_PARAM;
extern float INTER_PARAM;
extern float CROSSWALK_DETECTION_X1;
extern float CROSSWALK_DETECTION_X2;
extern float CROSSWALK_DETECTION_Y1;
extern float CROSSWALK_DETECTION_Y2;
extern int CROSSWALK_DETECTION_RECT_HEIGHT_THRESHOLD;
extern int CROSSWALK_DETECTION_RECT_WIDTH_THRESHOLD;
extern int CROSSWALK_DETECTION_RECT_COUNT_THRESHOLD;
extern float STARTLINE_DETECTION_X1;
extern float STARTLINE_DETECTION_X2;
extern float STARTLINE_DETECTION_Y1;
extern float STARTLINE_DETECTION_Y2;
extern int STARTLINE_DETECTION_THRESHOLD;
extern int GFT_MAX_CORNER_QUANTITY;
extern float GFT_CORNER_QUALITY_LEVEL;
extern int GFT_MIN_CORNER_DISTANCE;
extern float STEERING_OFFSET;
extern float STEERING_OFFSET_2;
extern float STEERING_KI;
extern float STEERING_KD;

// 초기화 함수 선언
void load_constants(const std::string& path = "../constants.json");
