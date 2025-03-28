

#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#define MIN_AREA 500  // Set a reasonable threshold based on object size

using namespace std;
using namespace cv;
int main(int argc, char** argv) {
   VideoCapture video_load(0);//capturing video from default camera//
   namedWindow("Adjust");//declaring window to show the image//
   int Hue_Lower_Value = 0;//initial hue value(lower)//
   int Hue_Lower_Upper_Value = 26;//initial hue value(upper)//
   int Saturation_Lower_Value = 11;//initial saturation(lower)//
   int Saturation_Upper_Value = 99;//initial saturation(upper)//
   int Value_Lower = 212;//initial value (lower)//
   int Value_Upper = 255;//initial saturation(upper)//
   createTrackbar("Hue_Lower", "Adjust", &Hue_Lower_Value, 255);//track-bar for lower hue//
   createTrackbar("Hue_Upper", "Adjust", &Hue_Lower_Upper_Value, 255);//track-bar for lower-upper hue//
   createTrackbar("Sat_Lower", "Adjust", &Saturation_Lower_Value, 255);//track-bar for lower saturation//
   createTrackbar("Sat_Upper", "Adjust", &Saturation_Upper_Value, 255);//track-bar for higher saturation//
   createTrackbar("Val_Lower", "Adjust", &Value_Lower, 255);//track-bar for lower value//
   createTrackbar("Val_Upper", "Adjust", &Value_Upper, 255);//track-bar for upper value//
   while (1) {
      Mat actual_Image;//matrix to load actual image//
      bool temp = video_load.read(actual_Image);//loading actual image to matrix from video stream//
      Mat convert_to_HSV;//declaring a matrix to store converted image//
      cvtColor(actual_Image, convert_to_HSV, COLOR_BGR2HSV);//converting BGR image to HSV and storing it in convert_to_HSV matrix//
      Mat detection_screen;//declaring matrix for window where object will be detected//
      inRange(convert_to_HSV,Scalar(Hue_Lower_Value,Saturation_Lower_Value, Value_Lower),Scalar(Hue_Lower_Upper_Value,Saturation_Upper_Value, Value_Upper), detection_screen);//applying track-bar modified value of track-bar//
      erode(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological opening for removing small objects from foreground//
      dilate(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological opening for removing small object from foreground//
      dilate(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological closing for filling up small holes in foreground//
      erode(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological closing for filling up small holes in foreground//

      cv::Mat mask = detection_screen.clone();

      // Draw the center point on the image
     // imshow("Threesholded Image", detection_screen);//showing detected object//
      //imshow("Original", actual_Image);//showing actual image//
      if (waitKey(30) == 27){ //if esc is press break the loop//
         break;
      }
      
      if (detection_screen.empty()) {
         std::cerr << "Error loading mask image!" << std::endl;
         return -1;
     }
 
     // Find contours
     std::vector<std::vector<cv::Point>> contours;
     cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 
     double max_area = 0;
     std::vector<cv::Point> largest_contour;
 
     // Iterate and find the largest contour with area above the threshold
     for (const auto& contour : contours) {
         double area = cv::contourArea(contour);
         if (area > MIN_AREA && area > max_area) {
             max_area = area;
             largest_contour = contour;
         }
     }
 
     if (!largest_contour.empty()) {
         // Get the enclosing circle
         cv::Point2f center;
         float radius;
         cv::minEnclosingCircle(largest_contour, center, radius);
 
         std::cout << "Center: (" << center.x << ", " << center.y << "), Radius: " << radius << std::endl;
 
         // Draw the detected object
         cv::Mat output;
         cv::cvtColor(mask, output, cv::COLOR_GRAY2BGR);
         cv::circle(output, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
 
         cv::imshow("Detected Object", output);
        // cv::waitKey(0);
         //cv::destroyAllWindows();
     } else {
         std::cout << "No large object detected." << std::endl;
     }
 
      // Display the result
      
   }
   return 0;
}