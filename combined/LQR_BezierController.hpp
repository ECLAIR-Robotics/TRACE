#ifndef LQR_BEZIER_CONTROLLER_HPP
#define LQR_BEZIER_CONTROLLER_HPP

#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

/**
 * @brief 2D Ball on Tilting Plate - LQR Controller with Bézier Waypoint Tracking
 * 
 * Implements:
 * - LQR control using pre-computed gain matrix K
 * - Alpha-beta filtering for noisy measurements
 * - Cubic Bézier curve trajectory generation
 * - Servo slew-rate limiting and saturation
 */
class LQRBezierController {
public:
    // State and control dimensions
    static constexpr int STATE_DIM = 6;  // [x, xdot, y, ydot, phi, theta]
    static constexpr int CONTROL_DIM = 2; // [phi_cmd, theta_cmd]
    static constexpr int MEAS_DIM = 2;    // [x, y]
    
    // Type aliases for clarity
    using State = std::array<double, STATE_DIM>;
    using Control = std::array<double, CONTROL_DIM>;
    using Measurement = std::array<double, MEAS_DIM>;
    using Waypoint = std::array<double, 2>; // [x, y]
    using KMatrix = std::array<std::array<double, STATE_DIM>, CONTROL_DIM>;
    
    /**
     * @brief Configuration parameters for the controller
     */
    struct Parameters {
        // Servo parameters
        double tau = 0.1;                    // servo time constant [s]
        double phi_max = 0.10472;            // max platform tilt [rad] (6 deg)
        double theta_max = 0.10472;          // max platform tilt [rad] (6 deg)
        double max_rate = 0.41888;           // max tilt speed [rad/s] (24 deg/s)
        
        // Alpha-beta filter parameters
        double alpha = 0.1;                  // position filter gain
        double beta = 0.01;                  // velocity filter gain
        
        // Trajectory parameters
        double kv_max = 0.15;                // max reference velocity [m/s]
        double total_time = 10.0;            // total time to traverse all waypoints [s]
        double start_threshold = 0.02;       // distance to start point before beginning trajectory [m]
        
        // System parameter
        double cg = (5.0/7.0) * 9.81;       // rolling factor * gravity
    };
    
    /**
     * @brief Constructor with waypoints vector
     * @param K Pre-computed LQR gain matrix (2x6)
     * @param waypoints Vector of waypoints defining the Bézier curve
     * @param params Controller parameters
     */
    LQRBezierController(const KMatrix& K, 
                       const std::vector<Waypoint>& waypoints,
                       const Parameters& params = Parameters())
        : K_(K), waypoints_(waypoints), params_(params) {
        
        if (waypoints_.empty()) {
            throw std::invalid_argument("Waypoints vector cannot be empty");
        }
        
        // Initialize filter state to zero
        x_hat_.fill(0.0);
        
        // Initialize previous control to zero
        u_prev_.fill(0.0);
        
        // Initialize time
        t_current_ = 0.0;
        
        // Initialize path tracking state
        path_started_ = false;
        
        // Initialize path tracking state
        path_started_ = false;
    }
    
    /**
     * @brief Constructor with CSV file
     * @param K Pre-computed LQR gain matrix (2x6)
     * @param csv_filepath Path to CSV file with waypoints
     * @param params Controller parameters
     */
    LQRBezierController(const KMatrix& K,
                       const std::string& csv_filepath,
                       const Parameters& params = Parameters())
        : K_(K), params_(params) {
        
        waypoints_ = loadWaypointsFromCSV(csv_filepath);
        
        if (waypoints_.empty()) {
            throw std::invalid_argument("No waypoints loaded from CSV file");
        }
        
        // Initialize filter state to zero
        x_hat_.fill(0.0);
        
        // Initialize previous control to zero
        u_prev_.fill(0.0);
        
        // Initialize time
        t_current_ = 0.0;
    }
    
    /**
     * @brief Compute control input for current measurement
     * @param measurement Current noisy measurement [x, y]
     * @param dt Time step since last control update [s]
     * @return Control input [phi_cmd, theta_cmd]
     */
    Control computeControl(const Measurement& measurement, double dt) {
        // 1. Update alpha-beta filter with new measurement
        updateFilter(measurement, dt);
        
        // 2. Compute desired state from Bézier trajectory
        State x_des = computeDesiredState(dt);
        
        // 3. Compute LQR control law: u = -K * (x_hat - x_des)
        Control u = computeLQRControl(x_des);
        
        // 4. Apply slew-rate limiting and saturation
        u = applySlewRateLimit(u, dt);
        u = applySaturation(u);
        
        // 5. Update previous control
        u_prev_ = u;
        
        return u;
    }
    
    /**
     * @brief Reset the controller state
     * @param initial_state Initial state estimate (optional, defaults to zero)
     */
    void reset(const State& initial_state = State{0}) {
        x_hat_ = initial_state;
        u_prev_.fill(0.0);
        t_current_ = 0.0;
        path_started_ = false;
    }
    
    /**
     * @brief Get current filtered state estimate
     */
    const State& getFilteredState() const {
        return x_hat_;
    }
    
    /**
     * @brief Get current time
     */
    double getCurrentTime() const {
        return t_current_;
    }
    
    /**
     * @brief Check if path traversal has started
     */
    bool hasPathStarted() const {
        return path_started_;
    }
    
    /**
     * @brief Update waypoints dynamically and reset trajectory time
     * @param waypoints New waypoint vector
     * @param reset_time If true (default), resets trajectory time to 0
     */
    void setWaypoints(const std::vector<Waypoint>& waypoints, bool reset_time = true) {
        if (waypoints.empty()) {
            throw std::invalid_argument("Waypoints vector cannot be empty");
        }
        waypoints_ = waypoints;
        
        if (reset_time) {
            t_current_ = 0.0;
            path_started_ = false;
        }
    }
    
    /**
     * @brief Load and set waypoints from CSV file, resetting trajectory time
     * @param filepath Path to CSV file
     */
    void loadAndSetWaypoints(const std::string& filepath) {
        auto waypoints = loadWaypointsFromCSV(filepath);
        setWaypoints(waypoints, true);
    }
    
    /**
     * @brief Load waypoints from CSV file
     * @param filepath Path to CSV file
     * @return Vector of waypoints
     * 
     * Expected CSV format:
     * x,y
     * 0.1,0.2
     * 0.3,0.4
     * ...
     * 
     * First line is treated as header and skipped.
     */
    static std::vector<Waypoint> loadWaypointsFromCSV(const std::string& filepath) {
        std::vector<Waypoint> waypoints;
        std::ifstream file(filepath);
        
        if (!file.is_open()) {
            throw std::runtime_error("Could not open CSV file: " + filepath);
        }
        
        std::string line;
        bool first_line = true;
        
        while (std::getline(file, line)) {
            // Skip header line
            if (first_line) {
                first_line = false;
                continue;
            }
            
            // Skip empty lines
            if (line.empty()) {
                continue;
            }
            
            // Parse x,y values
            std::stringstream ss(line);
            std::string x_str, y_str;
            
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    waypoints.push_back({x, y});
                } catch (const std::exception& e) {
                    throw std::runtime_error("Error parsing CSV line: " + line);
                }
            }
        }
        
        file.close();
        return waypoints;
    }

private:
    // Controller state
    KMatrix K_;                          // LQR gain matrix
    std::vector<Waypoint> waypoints_;    // Waypoint list
    Parameters params_;                   // Controller parameters
    State x_hat_;                        // Filtered state estimate
    Control u_prev_;                     // Previous control input
    double t_current_;                   // Current simulation time
    bool path_started_;                  // Whether path traversal has begun
    
    /**
     * @brief Update alpha-beta filter with new measurement
     */
    void updateFilter(const Measurement& y_meas, double dt) {
        // Predict position using velocity
        double x_pred = x_hat_[0] + x_hat_[1] * dt;
        double y_pred = x_hat_[2] + x_hat_[3] * dt;
        
        // Compute residual
        double res_x = y_meas[0] - x_pred;
        double res_y = y_meas[1] - y_pred;
        
        // Update position estimates
        x_hat_[0] = x_pred + params_.alpha * res_x;
        x_hat_[2] = y_pred + params_.alpha * res_y;
        
        // Update velocity estimates
        x_hat_[1] += (params_.beta / dt) * res_x;
        x_hat_[3] += (params_.beta / dt) * res_y;
        
        // Use commanded control inputs as platform angle estimates
        // This assumes servos track commands reasonably well
        // Can be replaced with encoder readings if available
        x_hat_[4] = u_prev_[0];  // phi estimate from phi_cmd
        x_hat_[5] = u_prev_[1];  // theta estimate from theta_cmd
    }
    
    /**
     * @brief Compute desired state from Bézier trajectory
     * 
     * Only increments trajectory time once the ball is within start_threshold
     * of the first waypoint. This prevents cutting off the beginning of a path
     * while the ball is moving to the start position.
     */
    State computeDesiredState(double dt) {
        // Get the starting position (first waypoint)
        Waypoint start_pos = waypoints_[0];
        
        // Check if we're close enough to start traversing
        double dist_to_start = sqrt(pow(x_hat_[0] - start_pos[0], 2) + 
                                   pow(x_hat_[2] - start_pos[1], 2));
        
        if (!path_started_) {
            if (dist_to_start < params_.start_threshold) {
                path_started_ = true;
            } else {
                // If not started yet, return the starting waypoint as desired state
                State x_des;
                x_des[0] = start_pos[0];
                x_des[1] = 0.0;  // zero velocity at start
                x_des[2] = start_pos[1];
                x_des[3] = 0.0;  // zero velocity at start
                x_des[4] = 0.0;
                x_des[5] = 0.0;
                return x_des;
            }
        }
        
        // Path has started - increment time and compute trajectory
        t_current_ += dt;
        
        int numPoints = waypoints_.size();
        int numSegments = numPoints; // Closed-loop
        
        // Compute global trajectory parameter
        double segments_per_second = numPoints / params_.total_time / 3.0;
        double t_global = t_current_ * segments_per_second;
        double t_mod = fmod(t_global, static_cast<double>(numSegments));
        
        // Segment index and local parameter
        int seg = static_cast<int>(floor(t_mod));
        double t_local = t_mod - floor(t_mod);
        
        // Get 4 control points with cyclic indexing
        std::array<Waypoint, 4> P;
        for (int i = 0; i < 4; ++i) {
            int idx = (4 * seg + i) % numPoints;
            P[i] = waypoints_[idx];
        }
        
        // Compute position and velocity on Bézier curve
        double x_wp, y_wp, xdot_wp, ydot_wp;
        bezierPosVel(P, t_local, x_wp, y_wp, xdot_wp, ydot_wp);
        
        // Scale velocity to kv_max
        double vel_mag = sqrt(xdot_wp * xdot_wp + ydot_wp * ydot_wp);
        double scale = std::min(1.0, params_.kv_max / (vel_mag + 1e-6));
        
        // Build desired state vector
        State x_des;
        x_des[0] = x_wp;
        x_des[1] = xdot_wp * scale;
        x_des[2] = y_wp;
        x_des[3] = ydot_wp * scale;
        x_des[4] = 0.0; // desired phi = 0
        x_des[5] = 0.0; // desired theta = 0
        
        return x_des;
    }
    
    /**
     * @brief Compute cubic Bézier position and velocity
     */
    void bezierPosVel(const std::array<Waypoint, 4>& P, double t,
                     double& x, double& y, double& xdot, double& ydot) const {
        // Cubic Bézier basis functions
        double t2 = t * t;
        double t3 = t2 * t;
        double mt = 1.0 - t;
        double mt2 = mt * mt;
        double mt3 = mt2 * mt;
        
        // Position
        x = mt3 * P[0][0] + 
            3.0 * mt2 * t * P[1][0] + 
            3.0 * mt * t2 * P[2][0] + 
            t3 * P[3][0];
            
        y = mt3 * P[0][1] + 
            3.0 * mt2 * t * P[1][1] + 
            3.0 * mt * t2 * P[2][1] + 
            t3 * P[3][1];
        
        // Velocity (first derivative)
        xdot = 3.0 * mt2 * (P[1][0] - P[0][0]) +
               6.0 * mt * t * (P[2][0] - P[1][0]) +
               3.0 * t2 * (P[3][0] - P[2][0]);
               
        ydot = 3.0 * mt2 * (P[1][1] - P[0][1]) +
               6.0 * mt * t * (P[2][1] - P[1][1]) +
               3.0 * t2 * (P[3][1] - P[2][1]);
    }
    
    /**
     * @brief Compute LQR control law: u = -K * (x_hat - x_des)
     */
    Control computeLQRControl(const State& x_des) const {
        State error;
        for (int i = 0; i < STATE_DIM; ++i) {
            error[i] = x_hat_[i] - x_des[i];
        }
        
        Control u;
        for (int i = 0; i < CONTROL_DIM; ++i) {
            u[i] = 0.0;
            for (int j = 0; j < STATE_DIM; ++j) {
                u[i] -= K_[i][j] * error[j];
            }
        }
        
        return u;
    }
    
    /**
     * @brief Apply slew-rate limiting to control input
     */
    Control applySlewRateLimit(const Control& u_desired, double dt) const {
        Control u_limited;
        double max_delta = params_.max_rate * dt;
        
        for (int i = 0; i < CONTROL_DIM; ++i) {
            double delta = u_desired[i] - u_prev_[i];
            delta = std::max(std::min(delta, max_delta), -max_delta);
            u_limited[i] = u_prev_[i] + delta;
        }
        
        return u_limited;
    }
    
    /**
     * @brief Apply saturation limits to control input
     */
    Control applySaturation(const Control& u) const {
        Control u_sat;
        u_sat[0] = std::max(std::min(u[0], params_.phi_max), -params_.phi_max);
        u_sat[1] = std::max(std::min(u[1], params_.theta_max), -params_.theta_max);
        return u_sat;
    }
};

#endif // LQR_BEZIER_CONTROLLER_HPP