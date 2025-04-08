#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#define PCA9685_ADDR 0x40
#define LED0_ON_L 0x06
#define MODE1 0x00
#define PRESCALE 0xFE

//ang must be in radians
double calcAngle(double ang){
    //Distance from servo motor to contact point
    double dist1 = 5.5;
    //Distance from point of conacts to the center of the base (axis of rotation assuming both ends move)
    double dist2 = 5;
    //Calculates the second, outer angle from motor based on interior platform angle from middle
    return atan(dist2 / dist1 * tan(ang));
}

// Writes a value to a PCA9685 register
void writeRegister(int fd, int reg, int val) {
    char buffer[2] = {static_cast<char>(reg), static_cast<char>(val)};
    write(fd, buffer, 2);
}

// Sets a PWM signal on a PCA9685 channel
void setPWM(int fd, int channel, int on, int off) {
    int base = LED0_ON_L + 4 * channel;
    char buffer[5] = {
        static_cast<char>(base),
        static_cast<char>(on & 0xFF),
        static_cast<char>((on >> 8) & 0xFF),
        static_cast<char>(off & 0xFF),
        static_cast<char>((off >> 8) & 0xFF),
    };
    write(fd, buffer, 5);
}

// Converts angle (0-180) to pulse width in PCA9685 steps
int angleToPWM(int angle) {
    int pulse_us = 500 + (angle * 11.11); // 500–2500 µs
    int pwm_val = static_cast<int>((pulse_us / 20000.0) * 4096); // Convert to 0–4095
    return pwm_val;
}

int main() {
    // Open I2C connection
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return 1;
    }

    // Connect to PCA9685
    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "Failed to connect to PCA9685\n";
        return 1;
    }

    // Configure PCA9685
    writeRegister(fd, MODE1, 0x10);     // Go to sleep
    writeRegister(fd, PRESCALE, 121);   // Set freq to ~50Hz
    writeRegister(fd, MODE1, 0x20);     // Restart with auto-increment

    // Continuous control loop
    int base_angle = 90;

    //These are the input angles
    int angle_NS, angle_WE = 0;

    //Assume 0=N, 1=W, 2=S, 3=E, positive angle_NS assumes tilting up at north, positive angle_WE assumes tilting up at west
    while (true) {
        //Calculates the output angle from the input angle
        angle_NS = calcAngle(angle_NS);
        angle_WE = calcAngle(angle_WE);

        //Controls each servo individually
        for (int channel = 0; channel < 4; ++channel) {
            switch(channel){
                case(0):
                    //Calculates pulsewidth from angle and sets it
                    int pwm = angleToPWM(base_angle + angle_NS);
                    setPWM(fd, channel, 0, pwm);
                    break;
                case(1):
                    int pwm = angleToPWM(base_angle + angle_WE);
                    setPWM(fd, channel, 0, pwm);
                    break;
                case(2):
                    int pwm = angleToPWM(base_angle - angle_NS);
                    setPWM(fd, channel, 0, pwm);
                    break;
                case(3):
                    int pwm = angleToPWM(base_angle - angle_WE);
                    setPWM(fd, channel, 0, pwm);
                    break;
                default:
                    //what abosulte tomfuckery occurred to make this happen
                    break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    close(fd);
    return 0;
}