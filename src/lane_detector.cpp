#include "lane_detector.hpp"
#include "constants.hpp"
#include <iostream>
#include <numeric>
#include <cmath>
#include <algorithm>  // std::sort
#include <numeric>    // std::accumulate
#include <cmath>  // std::sqrt

LaneDetector::LaneDetector() {}

int LaneDetector::process(const cv::Mat& frame, cv::Mat& vis_out) {
    if (frame.empty()) {
        std::cerr << "[LaneDetector] 입력 프레임이 비어있습니다." << std::endl;
        return 0;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    const cv::Mat& h = channels[0];
    const cv::Mat& s = channels[1];
    const cv::Mat& v = channels[2];

    int height = frame.rows;
    int width = frame.cols;
    int center_x = width / 2;
    float steer_value = 0.0f;

    // 관심영역 마스크
    cv::Mat roi_mask = createTrapezoidMask(height, width);
    cv::Mat valid_mask = (v >= VALID_V_MIN) & roi_mask;
    cv::Mat white_mask = (s < WHITE_S_MAX) & (v >= WHITE_V_MIN) & valid_mask;
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);
    yellow_pixel_count_ = cv::countNonZero(yellow_mask);
    
    vis_out = frame.clone();

    // WHITE_LINE_DRIVE에 따라 사용할 마스크 선택
    const cv::Mat& lane_mask = WHITE_LINE_DRIVE ? white_mask : yellow_mask;

    // 화면을 t개로 분할해서 오른쪽 차선 중심점 계산
    int t = NUM_DIVISIONS;  // 나중에 파라미터 추가
    std::vector<cv::Point> right_centers = getRightLaneCenters(lane_mask, t);

    // 계산된 중심점들을 lane_points에 추가하고 시각화
    std::vector<cv::Point> lane_points;
    for (const auto& pt : right_centers) {
        lane_points.emplace_back(pt);
        cv::circle(vis_out, pt, 3, cv::Scalar(0, 255, 0), -1);
    }

    // 5개 미만이면 우회전 강제
    if (static_cast<int>(right_centers.size()) < 5) {
        return static_cast<float>(FORCE_STEER_RIGHT);
    }
    else{
        // computeRightLaneRegression() 호출로 얻은 기울기(m)와 절편(b)
        auto [m, b] = computeRightLaneRegression(right_centers);

        // 1) 회귀 직선 x = m*y + b 와 화면 하단(y = height)과의 교점
        int y_bot = height;  // 320x200이면 y_bot == 200
        float intersection_x = m * y_bot + b;

        // 2) 목표선(TARGET_INTERSECTION_X)과의 차이 → 조향 에러
        //    (positive 이면 오른쪽으로, negative 이면 왼쪽으로 조향)
        float steer_value = intersection_x - static_cast<float>(TARGET_INTERSECTION_X);
        if (isCurve(right_centers))
        {
            return steer_value * CURVE_KP;
        }
        else
        {
            return steer_value * STRAIGHT_KP;
        }
        
    }
}

int LaneDetector::getYellowPixelCount() const {
    return yellow_pixel_count_;
}

// 여러 y값에 대해 오른쪽 차선의 중심점을 구하는 내부 헬퍼 함수
// mask: 추출된 흰색/노란색 마스크
// t: 화면을 t개 구간으로 나누어 계산
// 반환: (x, y) 좌표 벡터
std::vector<cv::Point> LaneDetector::getRightLaneCenters(const cv::Mat& mask, int t) {
    int height = mask.rows;
    int width  = mask.cols;
    std::vector<cv::Point> centers;
    centers.reserve(t);

    for (int i = 1; i <= t; ++i) {
        int y = static_cast<int>(height * (static_cast<float>(i) / (t + 1)));
        const uchar* row_ptr = mask.ptr<uchar>(y);

        // findBlobs를 활용해 연속 픽셀 블롭 찾기
        auto blobs = findBlobs(row_ptr, width);
        if (blobs.empty()) {
            // 블롭이 없으면 중앙 fallback
            centers.emplace_back(width/2, y);
        } else {
            // 가장 오른쪽 블롭 선택
            auto right_it = std::max_element(blobs.begin(), blobs.end(),
                [](auto& a, auto& b){ return a.back() < b.back(); });
            int sum_x = std::accumulate(right_it->begin(), right_it->end(), 0);
            int cx = sum_x / static_cast<int>(right_it->size());
            centers.emplace_back(cx, y);
        }
    }

    return centers;
}

std::vector<std::vector<int>> LaneDetector::findBlobs(
    const uchar* row_ptr,
    int width,
    int min_blob_size)
{
    std::vector<std::vector<int>> blobs;
    std::vector<int> current_blob;

    for (int x = 0; x < width; ++x) {
        if (row_ptr[x]) {
            // 연속된 1 픽셀 탐지 중
            current_blob.push_back(x);
        } else if (!current_blob.empty()) {
            // 연속 구간이 끝났을 때 최소 크기 이상이면 blobs에 추가
            if (static_cast<int>(current_blob.size()) >= min_blob_size) {
                blobs.push_back(current_blob);
            }
            current_blob.clear();
        }
    }
    // 마지막 블롭 처리
    if (!current_blob.empty() &&
        static_cast<int>(current_blob.size()) >= min_blob_size)
    {
        blobs.push_back(current_blob);
    }
    return blobs;
}

cv::Mat LaneDetector::createTrapezoidMask(int height, int width) {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
    int y_top = static_cast<int>(height * Y_TOP);
    int x_center = width / 2;
    int long_half = width * LONG_HALF;
    int short_half = static_cast<int>(width * SHORT_HALF);
    std::vector<cv::Point> pts = {
        {x_center - long_half, height}, 
        {x_center + long_half, height}, 
        {x_center + short_half, y_top}, 
        {x_center - short_half, y_top}};
    cv::fillConvexPoly(mask, pts, 255);
    if (ROI_REMOVE_LEFT)
        cv::rectangle(mask, 
            cv::Point(0, 0), 
            cv::Point(ROI_REMOVE_LEFT_X_THRESHOLD, height), 
            0, 
            cv::FILLED);
    return mask;
}

std::pair<float, float> LaneDetector::computeRightLaneRegression(
    const std::vector<cv::Point>& right_centers)
{

    // y값 기준으로 내림차순 정렬 → 가장 아래 5개 선택
    std::vector<cv::Point> pts = right_centers;
    std::sort(pts.begin(), pts.end(),
        [](const cv::Point& a, const cv::Point& b) {
            return a.y > b.y;
        });
    pts.resize(5);

    // 회귀 계산: x = m * y + b
    float sumY  = 0.0f, sumX = 0.0f, sumYY = 0.0f, sumXY = 0.0f;
    for (const auto& p : pts) {
        sumY  += p.y;
        sumX  += p.x;
        sumYY += static_cast<float>(p.y) * p.y;
        sumXY += static_cast<float>(p.y) * p.x;
    }
    const float N = 5.0f;
    float denom = (N * sumYY - sumY * sumY);
    if (std::abs(denom) < 1e-6f) {
        // 분모가 너무 작으면 수치 불안정 → 강제 우회전
        return { static_cast<float>(FORCE_STEER_RIGHT), 0.0f };
    }
    float m = (N * sumXY - sumY * sumX) / denom;
    float b = (sumX - m * sumY) / N;

    return { m, b };
}

bool LaneDetector::isCurve(const std::vector<cv::Point>& right_centers) const {
    const int N = static_cast<int>(right_centers.size());
    // 1차 회귀(m, b) 계산: x = m*y + b
    float sumY = 0, sumX = 0, sumYY = 0, sumXY = 0;
    for (auto& p : right_centers) {
        sumY  += p.y;
        sumX  += p.x;
        sumYY += float(p.y) * p.y;
        sumXY += float(p.y) * p.x;
    }
    float denom = N * sumYY - sumY * sumY;
    float m = (std::abs(denom) < 1e-6f)
        ? 0.0f
        : (N * sumXY - sumY * sumX) / denom;
    float b = (sumX - m * sumY) / N;

    // 잔차(RMSE) 계산
    float sumSqErr = 0;
    for (auto& p : right_centers) {
        float xPred = m * p.y + b;
        float e = p.x - xPred;
        sumSqErr += e * e;
    }
    float rmse = std::sqrt(sumSqErr / N);

    return rmse > CURVE_RMSE_THRESHOLD;
}