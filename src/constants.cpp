#include "constants.hpp"
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

namespace Constants {
    // 변수 정의
    int    FRAME_WIDTH;
    int    FRAME_HEIGHT;
    bool   VIEWER;

    int    ROI_Y_START;
    int    ROI_Y_END;
    bool   REMOVE_LEFT_ROI;
    int    REMOVE_LEFT_ROI_THRESHOLD;

    int    VALID_V_MIN;
    int    WHITE_S_MAX;
    int    WHITE_V_MIN;
    int    YELLOW_H_MIN;
    int    YELLOW_H_MAX;

    bool   WHITE_LINE_DRIVE;
    int    YELLOW_PIXEL_THRESHOLD;
    int    DEFAULT_LANE_GAP;
    float  AVG_PARAM;
    float  INTER_PARAM;

    float  STOPLINE_Y1;
    float  STOPLINE_Y2;
    float  STOPLINE_THRESHOLD;

    float  CROSSWALK_X1;
    float  CROSSWALK_X2;
    float  CROSSWALK_Y1;
    float  CROSSWALK_Y2;
    int    CROSSWALK_RECT_HEIGHT;
    int    CROSSWALK_RECT_WIDTH;
    int    CROSSWALK_COUNT_THRESHOLD;

    float  STARTLINE_X1;
    float  STARTLINE_X2;
    float  STARTLINE_Y1;
    float  STARTLINE_Y2;
    int    STARTLINE_THRESHOLD;

    int    GFT_MAX_CORNERS;
    float  GFT_QUALITY_LEVEL;
    int    GFT_MIN_DISTANCE;

    FollowLaneMode follow_lane_mode;
    float          lane_follow_steering_bias;

    float  STEERING_OFFSET;
    float  STEERING_OFFSET_2;
    float  STEERING_KP;
    float  STEERING_KI;
    float  STEERING_KD;
    float  THROTTLE_KP;
    float  BASE_THROTTLE;
    float  BASE_THROTTLE_LOW;
    float  BASE_THROTTLE_YELLOW;
    float  MAX_THROTTLE;

    void loadConstants(const std::string& path) {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("constants.json 열기 실패: " + path);
        json j; in >> j;

        // 영상 및 뷰어 관련
        FRAME_WIDTH            = j.value("FRAME_WIDTH", 320);
        FRAME_HEIGHT           = j.value("FRAME_HEIGHT", 200);
        VIEWER                 = j.value("VIEWER", true);

        // ROI 관련
        ROI_Y_START            = j.value("ROI_Y_START", 100);
        ROI_Y_END              = j.value("ROI_Y_END", 200);
        REMOVE_LEFT_ROI        = j.value("REMOVE_LEFT_ROI", false);
        REMOVE_LEFT_ROI_THRESHOLD = j.value("REMOVE_LEFT_ROI_THRESHOLD", 0);

        // 색상 임계값
        VALID_V_MIN            = j.value("VALID_V_MIN", 90);
        WHITE_S_MAX            = j.value("WHITE_S_MAX", 80);
        WHITE_V_MIN            = j.value("WHITE_V_MIN", 120);
        YELLOW_H_MIN           = j.value("YELLOW_H_MIN", 10);
        YELLOW_H_MAX           = j.value("YELLOW_H_MAX", 50);

        // 차선 추적 설정
        WHITE_LINE_DRIVE       = j.value("WHITE_LINE_DRIVE", true);
        YELLOW_PIXEL_THRESHOLD = j.value("YELLOW_PIXEL_THRESHOLD", 1200);
        DEFAULT_LANE_GAP       = j.value("DEFAULT_LANE_GAP", 430);
        AVG_PARAM              = j.value("AVG_PARAM", 0.5f);
        INTER_PARAM            = j.value("INTER_PARAM", 0.0f);

        // 정지선 감지
        STOPLINE_Y1            = j.value("STOPLINE_Y1", 0.5f);
        STOPLINE_Y2            = j.value("STOPLINE_Y2", 0.95f);
        STOPLINE_THRESHOLD     = j.value("STOPLINE_THRESHOLD", 0.2f);

        // 횡단보도 감지
        CROSSWALK_X1           = j.value("CROSSWALK_X1", 0.2f);
        CROSSWALK_X2           = j.value("CROSSWALK_X2", 0.8f);
        CROSSWALK_Y1           = j.value("CROSSWALK_Y1", 0.1f);
        CROSSWALK_Y2           = j.value("CROSSWALK_Y2", 0.6f);
        CROSSWALK_RECT_HEIGHT  = j.value("CROSSWALK_RECT_HEIGHT", 20);
        CROSSWALK_RECT_WIDTH   = j.value("CROSSWALK_RECT_WIDTH", 80);
        CROSSWALK_COUNT_THRESHOLD = j.value("CROSSWALK_COUNT_THRESHOLD", 3);

        // 출발선 감지
        STARTLINE_X1           = j.value("STARTLINE_X1", 0.2f);
        STARTLINE_X2           = j.value("STARTLINE_X2", 0.8f);
        STARTLINE_Y1           = j.value("STARTLINE_Y1", 0.4f);
        STARTLINE_Y2           = j.value("STARTLINE_Y2", 1.0f);
        STARTLINE_THRESHOLD    = j.value("STARTLINE_THRESHOLD", 80);

        // 특징점 감지
        GFT_MAX_CORNERS        = j.value("GFT_MAX_CORNERS", 100);
        GFT_QUALITY_LEVEL      = j.value("GFT_QUALITY_LEVEL", 0.01f);
        GFT_MIN_DISTANCE       = j.value("GFT_MIN_DISTANCE", 10);

        // 차선 추적 모드
        {
            std::string mode = j.value("follow_lane_mode", "CENTER");
            if      (mode == "RIGHT") follow_lane_mode = FollowLaneMode::RIGHT;
            else if (mode == "LEFT")  follow_lane_mode = FollowLaneMode::LEFT;
            else                        follow_lane_mode = FollowLaneMode::CENTER;
        }

        // 제어 값들
        lane_follow_steering_bias = j.value("lane_follow_steering_bias", 0.10f);
        STEERING_OFFSET         = j.value("STEERING_OFFSET", -0.25f);
        STEERING_OFFSET_2       = j.value("STEERING_OFFSET_2", -0.27f);
        STEERING_KP             = j.value("STEERING_KP", 0.018f);
        STEERING_KI             = j.value("STEERING_KI", 0.02f);
        STEERING_KD             = j.value("STEERING_KD", 0.05f);
        THROTTLE_KP             = j.value("THROTTLE_KP", 0.005f);
        BASE_THROTTLE           = j.value("BASE_THROTTLE", 0.4f);
        BASE_THROTTLE_LOW      = j.value("BASE_THROTTLE_LOW", 0.2f);
        BASE_THROTTLE_YELLOW   = j.value("BASE_THROTTLE_YELLOW", 0.3f);
        MAX_THROTTLE            = j.value("MAX_THROTTLE", 0.8f);
    }
}
