#include"MotorController.h"
#include"ESP32Servo.h"

MotorController::MotorController(unsigned char pwmPin){
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  this->pwmControl = new Servo();
  this->pwmControl->attach(pwmPin);
}

void MotorController::setPower(double power){
  if(power > 1){
    this->pwmControl->writeMicroseconds(2000);
  } else if(power < -1){
    this->pwmControl->writeMicroseconds(1000);
  } else {
    unsigned int microsecondsValue = map((power * 10000), -10000, 10000, 1000, 2000);
    this->pwmControl->writeMicroseconds(microsecondsValue);
  }
}
