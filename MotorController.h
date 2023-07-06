#ifndef motor_controller_h
#define motor_controller_h

#include"ESP32Servo.h"

class MotorController {
public:
    MotorController(unsigned char pwmPin);
    void setPower(double power);

private:
    Servo *pwmControl;

};

#endif
