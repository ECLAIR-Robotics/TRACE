

#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "motorcode.cpp"
#include <cmath>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>

#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

// TODO: EDIT THESE VALUES TO THE CORRECT PORTS
#define NORTH 1
#define SOUTH 2
#define EAST 3
#define WEST 4

// TODO: FIND THE "NEUTRAL VALUES" FOR EACH MOTOR:
#define N_NORTH 10
#define N_SOUTH 10
#define N_EAST 10
#define N_WEST 10

int i2c_fd;
void i2c_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    if(write(i2c_fd, buffer, 2)!= 2){
      printf("failed to write on reg %d\n", int(reg));  
    }
    
}

void set_pwm_freq(float freq_hz) {
    float prescaleval = 25000000.0f;  // 25MHz
    prescaleval /= 4096.0f;
    prescaleval /= freq_hz;
    prescaleval -= 1.0f;
    uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5f));

    // 🔧 Fix: Read MODE1 register properly
    uint8_t oldmode;
    uint8_t reg = MODE1;
    write(i2c_fd, &reg, 1);
    read(i2c_fd, &oldmode, 1);

    i2c_write_byte(MODE1, (oldmode & 0x7F) | 0x10); // sleep
    i2c_write_byte(PRESCALE, prescale);
    i2c_write_byte(MODE1, oldmode);
    usleep(5000);
    i2c_write_byte(MODE1, oldmode | 0xA1); // auto increment on
}


void set_pwm(int channel, int on, int off) {
    i2c_write_byte(LED0_ON_L + 4 * channel, on & 0xFF);
    i2c_write_byte(LED0_ON_L + 4 * channel + 1, on >> 8);
    i2c_write_byte(LED0_ON_L + 4 * channel + 2, off & 0xFF);
    i2c_write_byte(LED0_ON_L + 4 * channel + 3, off >> 8);
}

void set_servo_angle(int channel, float angle_deg) {
    

    int min_pulse = 205;  // 1.0 ms pulse
    int max_pulse = 410;  // 2.0 ms pulse

    int pulse = static_cast<int>(min_pulse + (angle_deg / 90.0f) * (max_pulse - min_pulse));
    //printf("setting angle to %f, using a pulse length of %d\n", angle_deg, pulse);
    set_pwm(channel, 0, pulse);
}

float translate_angle(float tilt_angle) {
    float max_tilt = 9.5;
    float platform_radius = 6;
    float arm_length = 5.625;
    tilt_angle = max_tilt > tilt_angle ? tilt_angle : max_tilt;
    tilt_angle = -max_tilt < tilt_angle ? tilt_angle : -max_tilt;
    return tilt_angle * (platform_radius / arm_length);
}

int init_motors() {
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C\n";
        return 1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "Failed to set I2C address\n";
        return 1;
    }

    set_pwm_freq(50); // 50Hz for servos
    return 0;
}

#include <chrono>
#include <algorithm>

class PID {
public:
    PID(double kp, double ki, double kd,
        double min_output = -std::numeric_limits<double>::infinity(),
        double max_output = std::numeric_limits<double>::infinity())
        : kp(kp), ki(ki), kd(kd),
          min_output(min_output), max_output(max_output),
          prev_error(0), integral(0),
          last_time(std::chrono::steady_clock::now()) {}

    double update(double setpoint, double measured) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_time;
        double dt = elapsed.count();
        last_time = now;

        double error = setpoint - measured;
        double derivative = (error - prev_error) / dt;

        // Tentative integral update
        double new_integral = integral + error * dt;

        // Tentative output with new integral
        double output = kp * error + ki * new_integral + kd * derivative;

        // Clamp output and apply anti-windup
        if (output > max_output) {
            output = max_output;
        } else if (output < min_output) {
            output = min_output;
        } else {
            integral = new_integral; // Only apply integral if not saturated
        }

        prev_error = error;
        return output;
    }

private:
    double kp, ki, kd;
    double min_output, max_output;
    double prev_error;
    double integral;
    std::chrono::steady_clock::time_point last_time;
};
    


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
   if (init_motors()) {
    return 1;
   }

   PID x_control(1,0,1, 9, -9);
   PID y_control(1,0,1, 9, -9);
   float set_x = 0;
   float set_y = 0;

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
         
        
         float angle_rad = 45 * M_PI / 180.0;
         
         float cos_theta = std::cos(angle_rad);
         floate sin_theta = std::sin(angle_rad);
        x = x * cos_theta - y * sin_theta,
        y = x * sin_theta + y * cos_theta,

         // Display the position of the ball
         std::string position_text = "Ball at " + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
         cv::putText(output, position_text, cv::Point(center_int.x, center_int.y + 20), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
         cv::imshow("Detected Object", output);
         
         // TODO: When ready for PID Control, uncomment this (2/?)
         float x_angle = x_control.update(set_x, x);
         float y_angle = y_control.update(set_y, y);
         
         set_servo_angle(NORTH, N_NORTH+y_angle); 
         set_servo_angle(SOUTH, N_SOUTH- y_angle);   
         set_servo_angle(EAST, N_EAST + x_angle);   
         set_servo_angle(WEST, N_WEST-x_angle); 

         
     } else {
         std::cout << "No large object detected." << std::endl;
     }
 
      // Display the result
      
   }
   return 0;
}