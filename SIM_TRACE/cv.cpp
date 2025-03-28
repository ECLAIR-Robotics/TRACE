

#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;
int main(int argc, char** argv) {
   VideoCapture video_load(0);//capturing video from default camera//
   namedWindow("Adjust");//declaring window to show the image//
   int Hue_Lower_Value = 0;//initial hue value(lower)//
   int Hue_Lower_Upper_Value = 22;//initial hue value(upper)//
   int Saturation_Lower_Value = 0;//initial saturation(lower)//
   int Saturation_Upper_Value = 255;//initial saturation(upper)//
   int Value_Lower = 0;//initial value (lower)//
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
      Moments m = moments(detection_screen, true);
      Point center(m.m10 / m.m00, m.m01 / m.m00);
      printf("%f\n", m.m10/m.m00);
      printf("%f\n", m.m01/m.m00);

      // Draw the center point on the image
      circle(actual_Image, center, 5, Scalar(0, 255, 0), -1);
      imshow("Threesholded Image", detection_screen);//showing detected object//
      imshow("Original", actual_Image);//showing actual image//
      if (waitKey(30) == 27){ //if esc is press break the loop//
         break;
      }
      
      
      // Display the result
      
   }
   return 0;
}