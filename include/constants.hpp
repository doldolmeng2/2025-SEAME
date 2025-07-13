// constants.hpp
#pragma once

// 프레임 해상도
constexpr int FRAME_WIDTH = 320;
constexpr int FRAME_HEIGHT = 200;

// 관심 영역 (ROI)
constexpr int ROI_Y_START = 100;
constexpr int ROI_Y_END = 200;

// HSV 차선 마스크 기준값
constexpr int WHITE_S_MAX = 80;
constexpr int WHITE_V_MIN = 120;
constexpr int VALID_V_MIN = 90;
constexpr int YELLOW_H_MIN = 20;
constexpr int YELLOW_H_MAX = 40;

// 디버깅 모드에서 시각화 표시 여부
constexpr bool VIEWER = true;
