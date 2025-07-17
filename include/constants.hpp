#pragma once
#include <string>
#include <nlohmann/json.hpp>

namespace Constants {
    // Follow-lane 모드 정의
    enum class FollowLaneMode {
        CENTER,
        LEFT,
        RIGHT
    };

    // DrivePhase 정의
    enum class DrivePhase {
        START,
        CENTRE_WHITE,
        STOP_CROSSWALK,
        RIGHT_WHITE,
        CENTRE_YELLOW,
        LEFT_YELLOW,
        RIGHT_WHITE_AFTER,
        FINISH
    };

    // Video & Viewer
    extern int    FRAME_WIDTH;
    extern int    FRAME_HEIGHT;
    extern bool   VIEWER;

    // Region of Interest
    extern int    ROI_Y_START;
    extern int    ROI_Y_END;
    extern bool   REMOVE_LEFT_ROI;
    extern int    REMOVE_LEFT_ROI_THRESHOLD;

    // Color thresholds
    extern int    VALID_V_MIN;
    extern int    WHITE_S_MAX;
    extern int    WHITE_V_MIN;
    extern int    YELLOW_H_MIN;
    extern int    YELLOW_H_MAX;

    // Lane tracking
    extern bool   WHITE_LINE_DRIVE;
    extern int    YELLOW_PIXEL_THRESHOLD;
    extern int    DEFAULT_LANE_GAP;
    extern float  AVG_PARAM;
    extern float  INTER_PARAM;

    // Stopline detection
    extern float  STOPLINE_Y1;
    extern float  STOPLINE_Y2;
    extern float  STOPLINE_THRESHOLD;

    // Crosswalk detection
    extern float  CROSSWALK_X1;
    extern float  CROSSWALK_X2;
    extern float  CROSSWALK_Y1;
    extern float  CROSSWALK_Y2;
    extern int    CROSSWALK_RECT_HEIGHT;
    extern int    CROSSWALK_RECT_WIDTH;
    extern int    CROSSWALK_COUNT_THRESHOLD;

    // Startline detection
    extern float  STARTLINE_X1;
    extern float  STARTLINE_X2;
    extern float  STARTLINE_Y1;
    extern float  STARTLINE_Y2;
    extern int    STARTLINE_THRESHOLD;

    // Feature detection
    extern int    GFT_MAX_CORNERS;
    extern float  GFT_QUALITY_LEVEL;
    extern int    GFT_MIN_DISTANCE;

    // Follow-lane 옵션
    extern FollowLaneMode follow_lane_mode;
    extern float          lane_follow_steering_bias;

    // Control gains & limits
    extern float  STEERING_OFFSET;
    extern float  STEERING_OFFSET_2;
    extern float  STEERING_KP;
    extern float  STEERING_KI;
    extern float  STEERING_KD;
    extern float  THROTTLE_KP;
    extern float  BASE_THROTTLE;
    extern float  BASE_THROTTLE_LOW;
    extern float  BASE_THROTTLE_YELLOW;
    extern float  MAX_THROTTLE;

    // JSON 에서 상수를 로드
    void loadConstants(const std::string& path);
}
