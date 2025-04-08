#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

//ang must be in radians
double calcAngle(double ang){
    //Distance from servo motor to contact point
    double dist1 = 5.5;
    //Distance from point of conacts to the center of the base (axis of rotation assuming both ends move)
    double dist2 = 5;
    //Calculates the second, outer angle from motor based on interior platform angle from middle
    return atan(dist2 / dist1 * tan(ang));
}

int main(){
    //1. Initialize pigpio
    if(gpioInitialise() < 0){
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return 1;
    }

    //Setting pin nums to rand vals, change later
    int gpioPin_North = 17;
    int gpioPin_West = 18;
    int gpioPin_South = 22;
    int gpioPin_East = 23;

    //Setting all motor pin nums to output
    gpioSetMode(gpioPin_North, PI_OUTPUT);
    gpioSetMode(gpioPin_West, PI_OUTPUT);
    gpioSetMode(gpioPin_South, PI_OUTPUT);
    gpioSetMode(gpioPin_East, PI_OUTPUT);

    //Control loop
    while(true){
        //Sets base angle to 90 because in middle of range (bi-directional movement)
        int base_angle = 90;

        //These will store the angle input from cv
        double angle_NS = 0;
        double angle_WE = 0;

        //Calculates output angle from input angle
        angle_NS = calcAngle(angle_NS);
        angle_WE = calcAngle(angle_WE);

        //Calculates the pulse width to send to the servo
        int pulseWidth_N = 500 + ((base_angle + angle_NS) * 11.11);
        int pulseWidth_W = 500 + ((base_angle + angle_WE) * 11.11);
        int pulseWidth_S = 500 + ((base_angle - angle_NS) * 11.11);
        int pulseWidth_E = 500 + ((base_angle - angle_WE) * 11.11);

        //Sends pulse width to servos (actual output part)
        gpioServo(gpioPin_North, pulseWidth_N);
        gpioServo(gpioPin_West, pulseWidth_W);
        gpioServo(gpioPin_South, pulseWidth_S);
        gpioServo(gpioPin_East, pulseWidth_E);
        
    }
    
    //Program cleanup
    gpioServo(gpioPin_East, 0);
    gpioServo(gpioPin_North, 0);
    gpioServo(gpioPin_South, 0);
    gpioServo(gpioPin_West, 0);
    gpioTerminate();
    return 0;
    
}