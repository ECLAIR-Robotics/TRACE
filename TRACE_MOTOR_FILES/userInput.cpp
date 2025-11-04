#include "input.hpp"

int main(){
  motor_driver* driver = new motor_driver(); 
  driver->add_motor(0);
  driver->move_motor_to_angle(30, 0);
  return 0; 
}