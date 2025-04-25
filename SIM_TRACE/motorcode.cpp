#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <iostream>

#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

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

    int pulse = static_cast<int>(min_pulse + (angle_deg / 180.0f) * (max_pulse - min_pulse));
    printf("setting angle to %f, using a pulse length of %d\n", angle_deg, pulse);
    set_pwm(channel, 0, pulse);
}

int main() {
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

while (true) {
    //set_servo_angle(0, 0);    // 0 degrees
    
    set_pwm(0,0,307);
    sleep(1);
    set_servo_angle(0, 90);   // 90 degrees
    set_pwm(0,0,205);
    sleep(1);
    set_servo_angle(0, 180);  // 180 degrees
    set_pwm(0,0,410);
    sleep(1);
    printf("vembis\n");
}


    close(i2c_fd);
    return 0;
}
