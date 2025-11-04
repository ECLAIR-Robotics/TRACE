#pragma once
#include <opencv2/opencv.hpp>

cv::Point getCenter(cv::Mat& image, cv::Point mark);
bool isBallColor(cv::Mat& image, int x, int y);
std::vector<cv::Point> generateSpiralGrid(int gridSize, int pixelsPerPoint);