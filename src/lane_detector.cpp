#include "lane_detector.hpp"
#include "constants.hpp"
#include <iostream>
#include <numeric>

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

    int height = frame.rows, width = frame.cols;

    // 관심영역 마스크 생성
    cv::Mat roi_mask = createTrapezoidMask(height, width);

    // 유효 마스크
    cv::Mat valid_mask = (v >= VALID_V_MIN) & roi_mask;
    cv::Mat white_mask = (s < WHITE_S_MAX) & (v >= WHITE_V_MIN) & valid_mask;
    cv::Mat yellow_mask = valid_mask & (~white_mask) & (h >= YELLOW_H_MIN) & (h <= YELLOW_H_MAX);

    // 흰색=255, 노란색=127
    cv::Mat grayscale = cv::Mat::zeros(v.size(), CV_8UC1);
    grayscale.setTo(255, white_mask);
    grayscale.setTo(127, yellow_mask);

    if (VIEWER) {
        cv::imshow("Grayscale Lane", grayscale);
        cv::waitKey(1);
    }

    vis_out = frame.clone();
    
    cv::Mat binary_white;
		white_mask.convertTo(binary_white, CV_8UC1, 255);

    // 차선 검출
    bool valid_left = false, valid_right = false;
    auto poly_left = slidingWindowDual(binary_white, "left", valid_left);
    auto poly_right = slidingWindowDual(binary_white, "right", valid_right);

    if (valid_left && valid_right) {
        bool valid_inter = false;
        cv::Point pt = computeIntersection(poly_left, poly_right, valid_inter);
        if (valid_inter) {
            int center_x = width / 2;
            int cross_point_offset = (pt.x - center_x) * 100 / center_x;
            cv::circle(vis_out, pt, 5, cv::Scalar(255, 0, 0), -1);
            cv::putText(vis_out, "x=" + std::to_string(pt.x), pt - cv::Point(0, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
            return cross_point_offset;
        }
    }

    return 0;
}

cv::Mat LaneDetector::createTrapezoidMask(int height, int width) {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);

    int y_top = static_cast<int>(height * 0.4);
    int x_center = width / 2;
    int long_half = width;
    int short_half = static_cast<int>(width * 0.3);

    std::vector<cv::Point> pts = {
        {x_center - long_half, height},
        {x_center + long_half, height},
        {x_center + short_half, y_top},
        {x_center - short_half, y_top}
    };

    cv::fillConvexPoly(mask, pts, 255);
    return mask;
}

cv::Vec2f LaneDetector::slidingWindowDual(const cv::Mat& binary, const std::string& side, bool& valid) {
    valid = false;
    int height = binary.rows;
    int width = binary.cols;

    // 히스토그램 생성 (하단 절반)
    cv::Mat hist;
    cv::reduce(binary.rowRange(height / 2, height), hist, 0, cv::REDUCE_SUM, CV_32S);
    int midpoint = width / 2;

    int base = (side == "left")
        ? std::distance(hist.begin<int>(), std::max_element(hist.begin<int>(), hist.begin<int>() + midpoint))
        : std::distance(hist.begin<int>() + midpoint, std::max_element(hist.begin<int>() + midpoint, hist.end<int>())) + midpoint;

    const int nwindows = 8, margin = 50, minpix = 50;
    int window_height = height / nwindows;

    std::vector<int> lane_x, lane_y;

    for (int w = 0; w < nwindows; ++w) {
        int win_y_low = height - (w + 1) * window_height;
        int win_y_high = height - w * window_height;
        int win_x_low = base - margin;
        int win_x_high = base + margin;

        for (int y = win_y_low; y < win_y_high; ++y) {
            for (int x = std::max(0, win_x_low); x < std::min(width, win_x_high); ++x) {
                if (binary.at<uchar>(y, x) > 0) {
                    lane_x.push_back(x);
                    lane_y.push_back(y);
                }
            }
        }

        if (lane_x.size() >= minpix)
            base = std::accumulate(lane_x.end() - minpix, lane_x.end(), 0) / minpix;
    }

    if (lane_x.size() < 200) {
        return (side == "left" && has_prev_left_) ? prev_poly_left_ :
               (side == "right" && has_prev_right_) ? prev_poly_right_ : cv::Vec2f(0, 0);
    }

    // 1차 선형 회귀
    std::vector<cv::Point> points;
    for (size_t i = 0; i < lane_x.size(); ++i) {
        points.emplace_back(lane_x[i], lane_y[i]);
    }

    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);

    float dx = line[0], dy = line[1], x0 = line[2], y0 = line[3];
    float m = dx / dy;
    float b = x0 - m * y0;

    float x_bottom = m * (height - 1) + b;
    if ((side == "left" && x_bottom > width / 2) || (side == "right" && x_bottom < width / 2)) {
        return (side == "left" && has_prev_left_) ? prev_poly_left_ :
               (side == "right" && has_prev_right_) ? prev_poly_right_ : cv::Vec2f(0, 0);
    }

    cv::Vec2f result(m, b);
    if (side == "left") {
        prev_poly_left_ = result;
        has_prev_left_ = true;
    } else {
        prev_poly_right_ = result;
        has_prev_right_ = true;
    }

    valid = true;
    return result;
}

cv::Point LaneDetector::computeIntersection(const cv::Vec2f& poly1, const cv::Vec2f& poly2, bool& valid) {
    float m1 = poly1[0], b1 = poly1[1];
    float m2 = poly2[0], b2 = poly2[1];

    if (std::abs(m1 - m2) < 1e-5) {
        valid = false;
        return cv::Point(0, 0);  // 평행한 경우
    }

    float y = (b2 - b1) / (m1 - m2);
    float x = m1 * y + b1;

    valid = true;
    return cv::Point(cvRound(x), cvRound(y));
}
