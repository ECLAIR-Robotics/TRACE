#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include "motor_driver.hpp"          // Your servo driver
#include "platform_controller.hpp"   // Platform tilt helper
#include "LQR_BezierController.hpp"  // LQR controller
#include "ball_cv.h"                 // Your CV functions

using namespace std;
using namespace std::chrono;

// Global flag for clean shutdown
volatile sig_atomic_t running = 1;

void signal_handler(int signum) {
    cout << "\nShutdown signal received. Stopping...\n";
    running = 0;
}

/**
 * @brief Get file modification time
 * @param filepath Path to file
 * @return Modification time in seconds since epoch, or 0 if file doesn't exist
 */
time_t getFileModTime(const string& filepath) {
    struct stat file_stat;
    if (stat(filepath.c_str(), &file_stat) == 0) {
        return file_stat.st_mtime;
    }
    return 0;
}

/**
 * @brief Ball position tracker using computer vision
 */
class BallTracker {
public:
    BallTracker(int camera_id = 2, int width = 1024, int height = 576, int padding = 224)
        : width_(width), height_(height), padding_(padding), 
          ball_found_(false), last_x_(0.0), last_y_(0.0) {
        
        // Camera calibration parameters
        fx_ = 534.15894866f;
        fy_ = 522.92638288f;
        cx_ = 340.66549491f;
        cy_ = 211.16012128f;
        real_diameter_ = 1.575f;  // Ball diameter in inches (convert to meters if needed)
        
        // Initialize camera
        cam_.open(camera_id, cv::CAP_V4L2);
        cam_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cam_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cam_.set(cv::CAP_PROP_FPS, 60);
        
        if (!cam_.isOpened()) {
            throw runtime_error("Could not open camera");
        }
        
        // Generate spiral grid for ball detection
        grid_points_ = generateSpiralGrid(8, 72);
        
        cout << "Camera initialized successfully" << endl;
    }
    
    ~BallTracker() {
        cam_.release();
    }
    
    /**
     * @brief Get current ball position
     * @return [x, y] position in meters (or your units), returns last known position if not found
     */
    LQRBezierController::Measurement getBallPosition() {
        cv::Mat img;
        cam_ >> img;
        
        if (img.empty()) {
            return {last_x_, last_y_};
        }
        
        // Crop image to remove padding
        img = img(cv::Rect(padding_, 0, width_ - padding_ * 2, height_));
        
        ball_found_ = false;
        
        // Search for ball using spiral grid
        for (const auto& point : grid_points_) {
            if (isBallColor(img, point.x, point.y)) {
                cv::Point center = getCenter(img, point);
                
                // Calculate 3D position
                int leftEdge = center.x, rightEdge = center.x;
                int centerY = center.y;
                
                // Find horizontal extent at center
                while (leftEdge > 0 && isBallColor(img, leftEdge - 1, centerY)) {
                    leftEdge -= 2;
                }
                while (rightEdge < width_ - padding_ * 2 && isBallColor(img, rightEdge + 1, centerY)) {
                    rightEdge += 2;
                }
                
                int diameter_px = rightEdge - leftEdge;
                float z = (fx_ * real_diameter_) / diameter_px;
                float x = (center.x - cx_) * z / fx_;
                float y = (center.y - cy_) * z / fy_;
                
                // Convert from inches to meters if needed
                // x /= 39.3701;  // Uncomment if real_diameter_ is in inches
                // y /= 39.3701;
                
                last_x_ = x;
                last_y_ = y;
                ball_found_ = true;
                
                break;  // Found ball, no need to continue
            }
        }
        
        return {last_x_, last_y_};
    }
    
    bool isBallFound() const {
        return ball_found_;
    }

private:
    cv::VideoCapture cam_;
    std::vector<cv::Point> grid_points_;
    int width_, height_, padding_;
    float fx_, fy_, cx_, cy_;
    float real_diameter_;
    bool ball_found_;
    double last_x_, last_y_;
    
    bool isBallColor(cv::Mat& image, int x, int y) {
        if (x < 0 || x >= image.cols || y < 0 || y >= image.rows) {
            return false;
        }
        cv::Vec3b color = image.at<cv::Vec3b>(y, x);
        int B = color[0], G = color[1], R = color[2];
        return (R > 100 && R > G + 30 && R > B + 30);
    }
    
    cv::Point getCenter(cv::Mat& image, cv::Point mark) {
        int top = mark.y, bottom = mark.y;
        while (top > 0 && isBallColor(image, mark.x, top - 1)) {
            top -= 2;
        }
        while (bottom < image.rows - 1 && isBallColor(image, mark.x, bottom + 1)) {
            bottom += 2;
        }
        int centerY = (top + bottom) / 2;
        
        int left = mark.x, right = mark.x;
        while (left > 0 && isBallColor(image, left - 1, mark.y)) {
            left -= 2;
        }
        while (right < image.cols - 1 && isBallColor(image, right + 1, mark.y)) {
            right += 2;
        }
        int centerX = (right + left) / 2;
        
        return cv::Point(centerX, centerY);
    }
};

int main(int argc, char* argv[]) {
    // Setup signal handler for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    cout << "=== Ball-on-Plate Control System ===" << endl;
    
    try {
        // ==================== INITIALIZE HARDWARE ====================
        
        // 1. Initialize motor driver (PCA9685)
        cout << "Initializing motor driver..." << endl;
        motor_driver driver(0x40);  // I2C address
        
        // 2. Configure motor channels
        // Phi axis: motors 0 (direct) and 2 (inverse)
        // Theta axis: motors 1 (direct) and 3 (inverse)
        PlatformController::MotorChannels channels;
        channels.phi_direct = 0;      // Motor 0: phi direct (90° + phi×11.6)
        channels.phi_inverse = 2;     // Motor 2: phi inverse (90° - phi×11.6)
        channels.theta_direct = 1;    // Motor 1: theta direct (90° + theta×11.6)
        channels.theta_inverse = 3;   // Motor 3: theta inverse (90° - theta×11.6)
        
        // 3. Configure platform parameters
        PlatformController::Parameters platform_params;
        platform_params.gear_ratio = 11.6;
        platform_params.neutral_angle_deg = 90.0;
        platform_params.phi_max = 0.10472;    // 6° in radians
        platform_params.theta_max = 0.10472;
        
        // Motor directions - inverse motors get negative multiplier by default
        // Adjust if motors move wrong way during testing
        platform_params.phi_direct_dir = 1.0;
        platform_params.phi_inverse_dir = -1.0;
        platform_params.theta_direct_dir = 1.0;
        platform_params.theta_inverse_dir = -1.0;
        
        // 4. Create platform controller
        cout << "Initializing platform controller..." << endl;
        PlatformController platform(&driver, channels, platform_params);
        
        // Set platform to level position
        cout << "Leveling platform..." << endl;
        platform.setLevel();
        this_thread::sleep_for(milliseconds(1000));
        
        // ==================== INITIALIZE CAMERA ====================
        
        cout << "Initializing camera..." << endl;
        BallTracker tracker(2, 1024, 576, 224);  // camera_id=2, with your resolution
        
        // ==================== INITIALIZE LQR CONTROLLER ====================
        
        // 5. Define LQR gain matrix K (from MATLAB simulation)
        // TODO: Replace with your actual K matrix from lqr() output
        LQRBezierController::KMatrix K = {{
            // phi gains:    [x,  xdot,  y,  ydot, phi, theta]
            {-5.0, -2.0,  0.0,  0.0,  1.0,  0.0},
            // theta gains:
            { 0.0,  0.0, -5.0, -2.0,  0.0,  1.0}
        }};
        
        // 6. Configure controller parameters
        LQRBezierController::Parameters lqr_params;
        lqr_params.alpha = 0.1;
        lqr_params.beta = 0.01;
        lqr_params.kv_max = 0.15;
        lqr_params.total_time = 10.0;
        lqr_params.start_threshold = 0.02;  // 2cm (adjust based on your units)
        
        // 7. Load waypoints and create controller
        cout << "Loading waypoints from CSV..." << endl;
        string waypoint_file = "bezier_points.csv";  // Use Bézier control points
        if (argc > 1) {
            waypoint_file = argv[1];
        }
        
        LQRBezierController controller(K, waypoint_file, lqr_params);
        cout << "Controller initialized with waypoints from: " << waypoint_file << endl;
        
        // 8. Setup file monitoring for automatic path reloading
        time_t last_file_mod_time = getFileModTime(waypoint_file);
        cout << "Monitoring '" << waypoint_file << "' for changes..." << endl;
        
        // ==================== CONTROL LOOP ====================
        
        cout << "\n=== Starting control loop ===" << endl;
        cout << "Press Ctrl+C to stop\n" << endl;
        
        const double target_dt = 0.01;  // 100 Hz control rate
        auto last_time = high_resolution_clock::now();
        
        int loop_count = 0;
        bool path_started_printed = false;
        int ball_lost_count = 0;
        int file_check_counter = 0;
        const int file_check_interval = 100;  // Check file every 100 loops (~1 second)
        
        while (running) {
            auto current_time = high_resolution_clock::now();
            double dt = duration<double>(current_time - last_time).count();
            
            // Maintain consistent loop rate
            if (dt < target_dt) {
                this_thread::sleep_for(microseconds(
                    static_cast<int>((target_dt - dt) * 1e6)
                ));
                current_time = high_resolution_clock::now();
                dt = duration<double>(current_time - last_time).count();
            }
            
            // 1. Get ball position from computer vision
            auto measurement = tracker.getBallPosition();
            
            // Track ball detection
            if (!tracker.isBallFound()) {
                ball_lost_count++;
                if (ball_lost_count % 50 == 0) {  // Print warning every ~0.5 seconds
                    cout << "Warning: Ball not detected, using last known position" << endl;
                }
            } else {
                ball_lost_count = 0;
            }
            
            // 2. Compute control input
            auto control = controller.computeControl(measurement, dt);
            
            // 3. Apply control to platform
            platform.setTilt(control[0], control[1]);  // phi, theta
            
            // 4. Check for path file updates (every ~1 second)
            file_check_counter++;
            if (file_check_counter >= file_check_interval) {
                file_check_counter = 0;
                time_t current_mod_time = getFileModTime(waypoint_file);
                
                if (current_mod_time != last_file_mod_time && current_mod_time != 0) {
                    cout << "\n>>> New path detected! Reloading waypoints... <<<" << endl;
                    try {
                        controller.loadAndSetWaypoints(waypoint_file);
                        last_file_mod_time = current_mod_time;
                        path_started_printed = false;  // Reset for new path
                        cout << ">>> Path reloaded successfully! <<<\n" << endl;
                    } catch (const exception& e) {
                        cerr << "Warning: Failed to reload path: " << e.what() << endl;
                    }
                }
            }
            
            // 5. Print status (every 100 loops = ~1 second)
            if (loop_count % 100 == 0) {
                auto state = controller.getFilteredState();
                printf("t=%.2f | pos=(%.4f, %.4f) | vel=(%.4f, %.4f) | ctrl=(%.4f, %.4f) | ball:%s\n",
                       controller.getCurrentTime(),
                       state[0], state[2],           // x, y position
                       state[1], state[3],           // x, y velocity
                       control[0], control[1],       // phi, theta commands
                       tracker.isBallFound() ? "✓" : "✗");
                
                // Print when path starts
                if (!path_started_printed && controller.hasPathStarted()) {
                    cout << ">>> Path traversal started! <<<" << endl;
                    path_started_printed = true;
                }
            }
            
            last_time = current_time;
            loop_count++;
        }
        
        // ==================== SHUTDOWN ====================
        
        cout << "\nShutting down..." << endl;
        
        // Return platform to level
        cout << "Returning platform to level..." << endl;
        platform.setLevel();
        this_thread::sleep_for(milliseconds(500));
        
        cout << "Shutdown complete." << endl;
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}