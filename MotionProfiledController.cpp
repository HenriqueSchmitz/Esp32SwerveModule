#include"MotionProfiledController.h"
#include"CustomEncoder.h"
#include"MotorController.h"
#include"VelocityControl.h"

MotionProfiledController::MotionProfiledController(MotorController *controller, CustomEncoder *encoder){
  this->controller = controller;
  this->encoder = encoder;
  this->velocityControl = new VelocityControl(this->controller, this->encoder);
  this->maxVelocity = 0;
  this->minVelocity = 0;
  this->maxAcceleration = 0;
  this->positiveTimeAccelerationMultiplier = 0;
  this->positivePositionAccelerationMultiplier = 0;
  this->minAcceleration = 0;
  this->negativeTimeAccelerationMultiplier = 0;
  this->negativePositionAccelerationMultiplier = 0;
  this->deadBand = 0;
  this->targetPosition = 0;
  this->motionStartTime = 0;
  this->isMotionDirectionPositive = true;
}

void MotionProfiledController::setPidConstants(double kp, double ki, double kd, double kf, double deadBand){
  this->velocityControl->setConstants(kp,ki,kd,kf,deadBand);
}

void MotionProfiledController::setMotionProfileConstants(double maxVelocity, double minVelocity, double maxAcceleration, double minAcceleration, double deadBand){
  this->maxVelocity = maxVelocity;
  this->minVelocity = minVelocity;
  this->maxAcceleration = maxAcceleration;
  this->positiveTimeAccelerationMultiplier = maxAcceleration / 1000000;
  this->positivePositionAccelerationMultiplier = -2 * maxAcceleration / minVelocity;
  this->minAcceleration = minAcceleration;
  this->negativeTimeAccelerationMultiplier = minAcceleration / 1000000;
  this->negativePositionAccelerationMultiplier = -2 * minAcceleration / maxVelocity;
  this->deadBand = deadBand;
}

void MotionProfiledController::setPosition(double targetPosition){
  unsigned long currentTime = micros();
  double currentPosition = this->encoder->getPosition();
  if(targetPosition != this->targetPosition){
    this->targetPosition = targetPosition;
    this->motionStartTime = currentTime;
    this->isMotionDirectionPositive = this->targetPosition >= currentPosition;
  }
  double error = this->targetPosition - currentPosition;
  if(error < this->deadBand && error > (-1 * this->deadBand)){
    this->velocityControl->setSepeed(0);
    Serial.print(" TargetSpeed:0");
  } else if(this->isMotionDirectionPositive){
    double riseSpeed = (currentTime - this->motionStartTime) * this->positiveTimeAccelerationMultiplier;
    double fallSpeed = error * this->negativePositionAccelerationMultiplier;
    double targetSpeed = min(min(riseSpeed, fallSpeed), this->maxVelocity);
    this->velocityControl->setSepeed(targetSpeed);
    Serial.print(" TargetSpeed:");
    Serial.print(targetSpeed);
  } else {
    double riseSpeed = (currentTime - this->motionStartTime) * this->negativeTimeAccelerationMultiplier;
    double fallSpeed = error * this->positivePositionAccelerationMultiplier;
    double targetSpeed = max(max(riseSpeed, fallSpeed), this->minVelocity);
    this->velocityControl->setSepeed(targetSpeed);
    Serial.print(" TargetSpeed:");
    Serial.print(targetSpeed);
  }
  this->velocityControl->refresh();
}
