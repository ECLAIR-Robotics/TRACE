

#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "motorcode.cpp"
#define MIN_AREA 500  // Set a reasonable threshold based on object size

using namespace std;
using namespace cv;
int main(int argc, char** argv) {
   VideoCapture video_load(0);//capturing video from default camera//
   namedWindow("Adjust");//declaring window to show the image//
   int Hue_Lower_Value = 0;//initial hue value(lower)//
   int Hue_Lower_Upper_Value = 26;//initial hue value(upper)//
   int Saturation_Lower_Value = 0;//initial saturation(lower)//
   int Saturation_Upper_Value = 8;//initial saturation(upper)//
   int Value_Lower = 212;//initial value (lower)//
   int Value_Upper = 255;//initial saturation(upper)//
   createTrackbar("Hue_Lower", "Adjust", &Hue_Lower_Value, 255);//track-bar for lower hue//
   createTrackbar("Hue_Upper", "Adjust", &Hue_Lower_Upper_Value, 255);//track-bar for lower-upper hue//
   createTrackbar("Sat_Lower", "Adjust", &Saturation_Lower_Value, 255);//track-bar for lower saturation//
   createTrackbar("Sat_Upper", "Adjust", &Saturation_Upper_Value, 255);//track-bar for higher saturation//
   createTrackbar("Val_Lower", "Adjust", &Value_Lower, 255);//track-bar for lower value//
   createTrackbar("Val_Upper", "Adjust", &Value_Upper, 255);//track-bar for upper value//
   // TODO: When ready for PID Control, uncomment this (1/?)
   // init_motors();
   while (1) {
      Mat actual_Image;//matrix to load actual image//
      bool temp = video_load.read(actual_Image);//loading actual image to matrix from video stream//
      Mat convert_to_HSV;//declaring a matrix to store converted image//
      cvtColor(actual_Image, convert_to_HSV, COLOR_BGR2HSV);//converting BGR image to HSV and storing it in convert_to_HSV matrix//
      Mat detection_screen;//declaring matrix for window where object will be detected//
      inRange(convert_to_HSV,Scalar(Hue_Lower_Value,Saturation_Lower_Value, Value_Lower),Scalar(Hue_Lower_Upper_Value,Saturation_Upper_Value, Value_Upper), detection_screen);//applying track-bar modified value of track-bar//
      cv::imshow("Actual Object", actual_Image);
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
     float ballRadius = .1;
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
    
         cv::minEnclosingCircle(largest_contour, center, radius);
         // Convert the center to an integer
         cv::Point center_int(center.x, center.y);

  
         cv::minEnclosingCircle(largest_contour, center, radius);
         // Convert the center to an integer
    
         int radius_int = radius;
         // Draw the enclosing circle on the original frame
         cv::circle(output, center_int, radius_int, cv::Scalar(0, 255, 0), 2);
         // Optionally, draw the contour for comparison
         cv::drawContours(output, contours, -1, cv::Scalar(0, 0, 255), 2);

         // Real-world diameter and camera parameters
         float real_diameter = 1.575;
         float fx = 647.07384177;
         float fy = 653.39571058;
         float cx = 353.31869253;
         float cy = 216.63488691;

         // Calculate the distance
         float px_diameter = 2 * radius_int;
         float z = (fx * real_diameter) / px_diameter;
         float x = (center_int.x - cx) * z / fx;
         float y = (center_int.y - cy) * z / fy;

         // Display the position of the ball
         std::string position_text = "Ball at " + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
         cv::putText(output, position_text, cv::Point(center_int.x, center_int.y + 20), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
         cv::imshow("Detected Object", output);
         
         // TODO: When ready for PID Control, uncomment this (2/?)
         
     } else {
         std::cout << "No large object detected." << std::endl;
     }
 
      // Display the result
      
   }
   return 0;
}