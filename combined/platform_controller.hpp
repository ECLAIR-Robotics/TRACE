#ifndef PLATFORM_CONTROLLER_HPP
#define PLATFORM_CONTROLLER_HPP

#include <cmath>
#include <array>
#include <stdexcept>

// Forward declaration - include your actual motor_driver header
class motor_driver;

/**
 * @brief Platform Tilt Controller
 * 
 * Converts platform tilt commands (phi, theta in radians) to individual
 * motor angle commands, accounting for:
 * - 11.6:1 gear ratio (platform angle to motor angle)
 * - Opposing motor motions for differential tilt
 * - Neutral position offsets
 */
class PlatformController {
public:
    /**
     * @brief Motor channel assignments on PCA9685
     */
    struct MotorChannels {
        int phi_direct;    // Motor that gets direct phi angle (positive side)
        int phi_inverse;   // Motor that gets inverse phi angle (negative side)
        int theta_direct;  // Motor that gets direct theta angle (positive side)
        int theta_inverse; // Motor that gets inverse theta angle (negative side)
    };
    
    /**
     * @brief Configuration parameters
     */
    struct Parameters {
        // Mechanical parameters
        double gear_ratio = 11.787;           // motor angle per platform angle
        double neutral_angle_deg = 90.0;    // motor angle for level platform
        
        // Safety limits (in radians for platform angles)
        double phi_max = 0.10472;           // ±6° in radians
        double theta_max = 0.10472;         // ±6° in radians
        
        // Motor direction multipliers (±1 to flip direction if needed)
        double phi_direct_dir = 1.0;      // Direction multiplier for phi direct motor
        double phi_inverse_dir = -1.0;    // Direction multiplier for phi inverse motor
        double theta_direct_dir = 1.0;    // Direction multiplier for theta direct motor
        double theta_inverse_dir = -1.0;  // Direction multiplier for theta inverse motor
    };
    
    /**
     * @brief Constructor
     * @param driver Pointer to initialized motor_driver
     * @param channels Motor channel assignments
     * @param params Configuration parameters
     */
    PlatformController(motor_driver* driver,
                      const MotorChannels& channels,
                      const Parameters& params = Parameters())
        : driver_(driver), channels_(channels), params_(params) {
        
        if (!driver_) {
            throw std::invalid_argument("Motor driver pointer cannot be null");
        }
    }
    
    /**
     * @brief Set platform tilt angles
     * @param phi Tilt angle around x-axis [radians]
     * @param theta Tilt angle around y-axis [radians]
     * @return 0 on success, negative on error
     * 
     * Motor configuration:
     * - Phi axis: motors 0 (direct) and 2 (inverse)
     * - Theta axis: motors 1 (direct) and 3 (inverse)
     * - Direct motors: angle = 90° + (tilt × 11.6)
     * - Inverse motors: angle = 90° - (tilt × 11.6)
     */
    int setTilt(double phi, double theta) {
        // Apply safety limits
        phi = clamp(phi, -params_.phi_max, params_.phi_max);
        theta = clamp(theta, -params_.theta_max, params_.theta_max);
        
        // Convert platform angles to motor angles (degrees)
        double phi_motor_offset = rad2deg(phi) * params_.gear_ratio;
        double theta_motor_offset = rad2deg(theta) * params_.gear_ratio;
        
        // Calculate individual motor angles
        // Phi axis (motors 0, 2)
        double phi_direct_angle = params_.neutral_angle_deg + 
                                  params_.phi_direct_dir * phi_motor_offset;
        double phi_inverse_angle = params_.neutral_angle_deg + 
                                   params_.phi_inverse_dir * phi_motor_offset;
        
        // Theta axis (motors 1, 3)
        double theta_direct_angle = params_.neutral_angle_deg + 
                                    params_.theta_direct_dir * theta_motor_offset;
        double theta_inverse_angle = params_.neutral_angle_deg + 
                                     params_.theta_inverse_dir * theta_motor_offset;
        
        // Send commands to motors
        int result = 0;
        result |= driver_->move_motor_to_angle(phi_direct_angle, channels_.phi_direct);
        result |= driver_->move_motor_to_angle(phi_inverse_angle, channels_.phi_inverse);
        result |= driver_->move_motor_to_angle(theta_direct_angle, channels_.theta_direct);
        result |= driver_->move_motor_to_angle(theta_inverse_angle, channels_.theta_inverse);
        
        return result;
    }
    
    /**
     * @brief Set platform to level (neutral) position
     * @return 0 on success, negative on error
     */
    int setLevel() {
        return setTilt(0.0, 0.0);
    }
    
    /**
     * @brief Get current parameters (for tuning/debugging)
     */
    const Parameters& getParameters() const {
        return params_;
    }
    
    /**
     * @brief Update parameters dynamically
     */
    void setParameters(const Parameters& params) {
        params_ = params;
    }

private:
    motor_driver* driver_;
    MotorChannels channels_;
    Parameters params_;
    
    /**
     * @brief Clamp value to range
     */
    double clamp(double value, double min_val, double max_val) const {
        return std::max(min_val, std::min(value, max_val));
    }
    
    /**
     * @brief Convert radians to degrees
     */
    double rad2deg(double rad) const {
        return rad * 180.0 / M_PI;
    }
};

#endif // PLATFORM_CONTROLLER_HPP