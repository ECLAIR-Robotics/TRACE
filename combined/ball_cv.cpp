#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

constexpr bool debug = false;
constexpr int gridSize = 8;
constexpr int pixelsPerPoint = 65;
constexpr float real_diameter = 1.575;
constexpr int width = 1024;
constexpr int height = 576;
constexpr int xPadding = 280;
constexpr int yPadding = 60;
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
// Camera Matrix:
//  [[534.15894866   0.         340.66549491]
//  [  0.         522.92638288 211.16012128]
//  [  0.           0.           1.        ]]
// Distortion Coefficients:
//  [[ 7.87054911e-02 -9.65068219e-01 -1.12876706e-03  7.65091028e-03
//    2.56899703e+00]]

int main() {
    cv::Point gridPoints[gridSize][gridSize];
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            gridPoints[y][x] = cv::Point(pixelsPerPoint / 2 + x * pixelsPerPoint,
                                         pixelsPerPoint / 2 + y * pixelsPerPoint);
        }
    }
    cv::Point lastBallGridIdx(gridSize / 2, gridSize / 2);

    cv::VideoCapture cam(2, cv::CAP_V4L2);
    cam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cam.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cam.set(cv::CAP_PROP_FPS, 60);
    if (!cam.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    } 

    cv::Mat img;
    while(true) {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        cam >> img;
        if (!img.empty()) {
            img = img(cv::Rect(0 + xPadding, yPadding , width - xPadding * 2, height - yPadding * 2));
            bool foundBall = false;
            int x = lastBallGridIdx.x;
            int y = lastBallGridIdx.y;
            cv::Point& startPoint = gridPoints[y][x];
            if (isBallColor(img, startPoint.x, startPoint.y)) {
                cv::Point center = getCenter(img, startPoint);
                cv::rectangle(img, center, cv::Point(center.x + 5, center.y + 5), 
                              cv::Scalar(255, 0, 0), 2, cv::LINE_8);
                foundBall = true;
                lastBallGridIdx.x = x;
                lastBallGridIdx.y = y;
            } else {
                int pointsChecked = 1;
                int steps = 1;
                if (debug) {
                    cv::rectangle(img, startPoint, cv::Point(startPoint.x + 5, startPoint.y + 5), 
                                cv::Scalar(pointsChecked * 4, 0, 0), 2, cv::LINE_8);
                }
                pointsChecked++;
                while (!foundBall) {
                    for (int dir = 0; dir < 4; dir++) {
                        for (int i = 0; i < steps; i++) {
                            x += directions[dir][0];
                            y += directions[dir][1];
                            if (x >= 0 && x < gridSize && y >= 0 && y < gridSize) {
                                cv::Point& point = gridPoints[y][x];
                                pointsChecked++;
                                if (isBallColor(img, point.x, point.y)) {
                                    cv::Point center = getCenter(img, point);
                                    if (debug) {
                                        cv::rectangle(img, point, cv::Point(point.x + 5, point.y + 5), 
                                                    cv::Scalar(pointsChecked * 4, 0, 0), 2, cv::LINE_8);
                                    }
                                    foundBall = true;
                                    lastBallGridIdx.x = x;
                                    lastBallGridIdx.y = y;
                                    break;
                                }
                                if (debug) {
                                    cv::rectangle(img, point, cv::Point(point.x + 5, point.y + 5), 
                                                cv::Scalar(0, 0, 0), 2, cv::LINE_8);
                                }
                            }
                            if (foundBall || pointsChecked == gridSize * gridSize) break;
                        }
                        if (foundBall || pointsChecked == gridSize * gridSize) break;
                        if (dir == 1 || dir == 3) steps++;
                    }
                    if (!debug && foundBall) break;
                }   
            }
            cv::imshow("camera", img);
        }                    
        if (cv::waitKey(1) >= 0) break;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "delta time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " microseconds" << std::endl;
    }
    cam.release();
    cv::destroyAllWindows();
    return 0;
}

cv::Point getCenter(cv::Mat& image, cv::Point mark) {
    // recall (0,0) is top left corner
    int top = mark.y, bottom = mark.y;
    while (top > 0 && isBallColor(image, mark.x, top - 1)) {
        top--;
    }
    while (bottom < height - 1 && isBallColor(image, mark.x, bottom + 1)) {
        bottom++;
    }
    if (debug) {
        cv::rectangle(image, cv::Point(mark.x, bottom), cv::Point(mark.x, top), cv::Scalar(255,255,255), 1, cv::LINE_8);
    }
    int centerY = (top + bottom) / 2;

    int left = mark.x, right = mark.x;
    while (left > 0 && isBallColor(image, left - 1, mark.y)) {
        left--;
    }
    while (right < width - 1 && isBallColor(image, right + 1, mark.y)) {
        right++;
    }
    int centerX = (right + left) / 2;

    if (debug) {
        cv::rectangle(image, cv::Point(right, mark.y), cv::Point(left, mark.y), cv::Scalar(255,255,255), 1, cv::LINE_8);
    }

    int leftCenterEdge = centerX, rightCenterEdge = centerX;
    while (leftCenterEdge > 0 && isBallColor(image, leftCenterEdge - 1, centerY)) {
        leftCenterEdge--;
    }
    while (rightCenterEdge < width - 1 && isBallColor(image, rightCenterEdge + 1, centerY)) {
        rightCenterEdge++;
    }
    int diameter = rightCenterEdge - leftCenterEdge;
    float z = (fx * real_diameter) / diameter;
    float x = (centerX - cx) * z / fx;
    float y = (centerY - cy) * z / fy;
    std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;
    return cv::Point(centerX, centerY);
}

bool isBallColor(cv::Mat& image, int x, int y) {
    cv::Vec3b color = image.at<cv::Vec3b>(y, x);
    // return 0.299 * color[2] + 0.587 * color[1] + 0.114 * color[0] < 120;
    // return (color[0] + color[1] + color[2]) / 3 < 90;
    int B = color[0], G = color[1], R = color[2];
    return (R > 100 && R > G + 30 && R > B + 30);
}