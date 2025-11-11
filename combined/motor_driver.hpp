// motor_driver.cpp
// PCA9685 + WiringPi servo driver (write pulse directly to PCA9685 registers).

#include <iostream>
#include <cmath>
#include <cstdint>
#include <unistd.h>      // usleep
#include <wiringPi.h>
#include <wiringPiI2C.h>

using std::cout;
using std::cerr;
using std::endl;

// ---------- User-configurable ----------
#define PCA9685_I2C_ADDR  0x40
#define PROJ_MOTOR_MAX 180.0f
#define PROJ_MOTOR_MIN 0.0f
#define SERVOMIN 1000  // µs
#define SERVOMAX 2000  // µs
const float OSC_CLOCK_HZ = 25'000'000.0f; // 25 MHz
// --------------------------------------

// PCA9685 register map and MODE bits
#define PCA9685_MODE1     0x00
#define PCA9685_MODE2     0x01
#define LED0_ON_L         0x06
#define LED0_ON_H         0x07
#define LED0_OFF_L        0x08
#define LED0_OFF_H        0x09
#define LEDALL_ON_L       0xFA
#define LEDALL_ON_H       0xFB
#define LEDALL_OFF_L      0xFC
#define LEDALL_OFF_H      0xFD
#define PCA9685_PRESCALE  0xFE

#define MODE1_RESTART     0x80
#define MODE1_AI          0x20
#define MODE1_SLEEP       0x10

#define MODE2_OUTDRV      0x04

#define PIN_ALL           16

class motor_driver
{
public:
    int i2c_fd;

    motor_driver(uint8_t i2c_addr = PCA9685_I2C_ADDR)
    : i2c_fd(-1), i2c_addr(i2c_addr), pwm_freq_hz(0.0f), period_us(0.0f)
    {
        i2c_fd = wiringPiI2CSetup(i2c_addr);
        if (i2c_fd < 0) {
            std::cerr << "[PCA9685] I2C open failed at addr 0x"
                      << std::hex << int(i2c_addr) << std::dec << "\n";
            return;
        }

        wiringPiI2CWriteReg8(i2c_fd, PCA9685_MODE2, MODE2_OUTDRV);

        int m1_read = wiringPiI2CReadReg8(i2c_fd, PCA9685_MODE1);
        if (m1_read < 0) m1_read = 0;
        uint8_t m1 = static_cast<uint8_t>(m1_read);
        wiringPiI2CWriteReg8(i2c_fd, PCA9685_MODE1, (m1 | MODE1_AI) & ~MODE1_SLEEP);

        set_pwm_freq(50.0f, OSC_CLOCK_HZ);

        wiringPiI2CWriteReg8(i2c_fd, LEDALL_ON_L,  0x00);
        wiringPiI2CWriteReg8(i2c_fd, LEDALL_ON_H,  0x00);
        wiringPiI2CWriteReg8(i2c_fd, LEDALL_OFF_L, 0x00);
        wiringPiI2CWriteReg8(i2c_fd, LEDALL_OFF_H, 0x00);
    }

    int move_motor_to_angle(float angle, int channel)
    {
        if (channel < 0 || (channel > 15 && channel != PIN_ALL)) {
            cerr << "[move_motor_to_angle] invalid channel: " << channel << "\n";
            return -1;
        }

        if (angle < PROJ_MOTOR_MIN) angle = PROJ_MOTOR_MIN;
        if (angle > PROJ_MOTOR_MAX) angle = PROJ_MOTOR_MAX;

        float pulse_us_f = map(angle, PROJ_MOTOR_MIN, PROJ_MOTOR_MAX, (float)SERVOMIN, (float)SERVOMAX);
        int pulse_us = static_cast<int>(std::round(pulse_us_f));
        return write_pulse_us(channel, pulse_us);
    }

    int write_pulse_us(int channel, int pulse_us)
    {
        if (channel < 0 || (channel > 15 && channel != PIN_ALL)) {
            cerr << "[write_pulse_us] invalid channel: " << channel << "\n";
            return -1;
        }
        if (pwm_freq_hz <= 0.0f || period_us <= 0.0f) {
            cerr << "[write_pulse_us] invalid PWM frequency/period (not initialized)\n";
            return -2;
        }
        if (pulse_us < 0) pulse_us = 0;

        // compute fraction of period and convert to 12-bit counts
        float fraction = (float)pulse_us / period_us;
        float raw = fraction * 4096.0f;
        int count = static_cast<int>(std::round(raw));

        if (count < 0) count = 0;
        if (count > 4095) count = 4095;


        // ON = 0, OFF = count  <-- FIXED: pass 'count' not 'pulse_us'
        pca9685PWMWrite(channel, 0, count);
        return 0;
    }

private:
    uint8_t i2c_addr;
    float pwm_freq_hz;
    float period_us;

    float map(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void set_pwm_freq(float freq_hz, float osc_clock_hz) {
        if (i2c_fd < 0) return;

        pwm_freq_hz = freq_hz;
        float prescaleval = osc_clock_hz / (4096.0f * freq_hz) - 1.0f;
        uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5f));

        int old_mode_read = wiringPiI2CReadReg8(i2c_fd, PCA9685_MODE1);
        if (old_mode_read < 0) old_mode_read = 0;
        uint8_t old_mode = static_cast<uint8_t>(old_mode_read);
        uint8_t sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP;

        wiringPiI2CWriteReg8(i2c_fd, PCA9685_MODE1, sleep_mode);
        wiringPiI2CWriteReg8(i2c_fd, PCA9685_PRESCALE, prescale);
        wiringPiI2CWriteReg8(i2c_fd, PCA9685_MODE1, old_mode);
        usleep(5000);

        wiringPiI2CWriteReg8(i2c_fd, PCA9685_MODE1, old_mode | MODE1_RESTART | MODE1_AI);

        period_us = 1e6f / freq_hz;
    }

    void pca9685PWMWrite(int pin, int on, int off)
    {
        int reg = baseReg(pin);

        uint16_t on12  = static_cast<uint16_t>(on & 0x0FFF);
        uint16_t off12 = static_cast<uint16_t>(off & 0x0FFF);

        wiringPiI2CWriteReg8(i2c_fd, reg    , on12  & 0xFF);
        wiringPiI2CWriteReg8(i2c_fd, reg + 1, (on12  >> 8) & 0x0F);
        wiringPiI2CWriteReg8(i2c_fd, reg + 2, off12 & 0xFF);
        wiringPiI2CWriteReg8(i2c_fd, reg + 3, (off12 >> 8) & 0x0F);
    }

    int baseReg(int pin)
    {
        return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
    }
};
