#ifndef velocity_control_h
#define velocity_control_h

#include"MotorController.h"
#include"CustomEncoder.h"

class VelocityControl {
public:
    VelocityControl(MotorController *motorController, CustomEncoder *encoder);
    void setConstants(double kp, double ki, double kd, double kf, double deadBand);
    void setSepeed(double targetSpeed);
    void refresh();

private:
    MotorController *motorController;
    CustomEncoder *encoder;
    double kp;
    double ki;
    double kd;
    double kf;
    double deadBand;
    
    double targetSpeed;
    double measuredSpeed;
    double error;
    unsigned long lastRefreshTime;
    double lastExecutionError;
    double integralAccumulator;

};

#endif
