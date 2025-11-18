#pragma once
#include <opencv2/opencv.hpp>

namespace ball_cv {
    constexpr int grid_size = 8;
    constexpr int pixels_per_point = 65;
    constexpr float real_diameter = 1.575; // Ball diameter in inches (convert to meters if needed)
    constexpr int width = 1024;
    constexpr int height = 576;
    constexpr int x_padding = 280;
    constexpr int y_padding = 60;
    constexpr float fx = 534.15894866;
    constexpr float fy = 522.92638288;
    constexpr float cx = 340.66549491;
    constexpr float cy = 211.16012128;
    constexpr int directions[4][2] = {
        {1, 0},
        {0, 1},
        {-1, 0},
        {0, -1}
    };
}


void getCenter(cv::Mat& image, cv::Point mark, double& last_x, double& last_y);
bool isBallColor(cv::Mat& image, int x, int y);