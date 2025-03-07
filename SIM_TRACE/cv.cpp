#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

int main() {
  // Open the default camera (usually the first camera)
  VideoCapture cap(0);

  if (!cap.isOpened()) {
    cout << "Error opening camera" << endl;
    return -1;
  }

  while (true) {
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    // Convert to HSV color space
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // Threshold the HSV image to get only orange colors
    Mat mask;
    int sensitivity = 5;
    inRange(hsv, Scalar(0, 0, 255-sensitivity), Scalar(255,sensitivity,255), mask);

    // Calculate moments of the binary image
    Moments m = moments(mask, true);

    // Get the center
    Point center(m.m10 / m.m00, m.m01 / m.m00);

    // === display the center. for debugging ===
    // Draw the center point on the frame
    circle(frame, center, 5, Scalar(0, 255, 0), -1);

    // Display the resulting frame
    imshow("Center Point", frame);

    // Press 'q' to exit the loop
    if (waitKey(30) == 'q')
      break;
  }

  // When everything done, release the video capture object
  cap.release();

  // Close all OpenCV windows
  destroyAllWindows();

  return 0;
}