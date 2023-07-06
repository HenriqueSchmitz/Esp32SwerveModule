#ifndef motion_profiled_controller_h
#define motion_profiled_controller_h

#include"CustomEncoder.h"
#include"MotorController.h"
#include"VelocityControl.h"

class MotionProfiledController {
public:
    MotionProfiledController(MotorController *controller, CustomEncoder *encoder);
    void setPidConstants(double kp, double ki, double kd, double kf, double deadBand);
    void setMotionProfileConstants(double maxVelocity, double minVelocity, double maxAcceleration, double minAcceleration, double deadBand);
    void setPosition(double targetPosition);

private:
    MotorController *controller;
    CustomEncoder *encoder;
    VelocityControl *velocityControl;

    double maxVelocity;
    double minVelocity;
    double maxAcceleration;
    double positiveTimeAccelerationMultiplier;
    double positivePositionAccelerationMultiplier;
    double minAcceleration;
    double negativeTimeAccelerationMultiplier;
    double negativePositionAccelerationMultiplier;
    double deadBand;

    double targetPosition;
    unsigned long motionStartTime;
    bool isMotionDirectionPositive;

};

#endif
