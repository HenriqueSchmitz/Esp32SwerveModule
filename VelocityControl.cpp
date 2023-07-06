#include"VelocityControl.h"
#include"MotorController.h"
#include"CustomEncoder.h"

VelocityControl::VelocityControl(MotorController *motorController, CustomEncoder *encoder){
  this->motorController = motorController;
  this->motorController->setPower(0);
  this->encoder = encoder;
  this->kp = 0;
  this->ki = 0;
  this->kd = 0;
  this->kf = 0;
  this->deadBand = 0;
  this->targetSpeed = 0;
  this->measuredSpeed = 0;
  this->error = 0;
  this->lastRefreshTime = micros();
  this->lastExecutionError = 0;
  this->integralAccumulator = 0;
}

void VelocityControl::setConstants(double kp, double ki, double kd, double kf, double deadBand){
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->kf = kf;
  this->deadBand = deadBand;
}

void VelocityControl::setSepeed(double targetSpeed){
  this->targetSpeed = targetSpeed;
}

void VelocityControl::refresh(){
  unsigned long currentTime = micros();
  this->measuredSpeed = this->encoder->getSpeed();
  this->error = this->targetSpeed - this->measuredSpeed;
  if(this->error < this->deadBand && this->error > (-1*this->deadBand)){
    this->error = 0;
  }
  double timeDelta = ((double)(currentTime - this->lastRefreshTime))/((double)1000000);
  double errorDelta = this->error - this->lastExecutionError;
  double p = this->kp * this->error;
  double i = this->ki * (this->error + this->lastExecutionError) * timeDelta / 2;
  this->integralAccumulator += i;
  double d = this->kd * errorDelta / timeDelta;
  double f = 0;
  if(this->targetSpeed > 0){
    f = this->kf;
  } else if(this->targetSpeed){
    f = -1 * this->kf;
  }
  double output = p + this->integralAccumulator + d + f;
//  Serial.print("p:");
//  Serial.print(p);
//  Serial.print(" i:");
//  Serial.print(this->integralAccumulator);
//  Serial.print(" d:");
//  Serial.print(d);
//  Serial.print(" f:");
//  Serial.print(f);
  Serial.print(" Output:");
  Serial.print(output);
  this->motorController->setPower(output);
  this->lastExecutionError = this->error;
  this->lastRefreshTime = currentTime;
}
