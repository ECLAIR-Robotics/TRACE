#include <pigpio.h>
#include <iostream>
#include <unistd.h>

int main(){
    //1. Initialize pigpio
    if(gpioInitialise() < 0){
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return 1;
    }

    
}