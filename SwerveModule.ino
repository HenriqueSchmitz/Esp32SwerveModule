#include"CustomEncoder.h"
#include"MotorController.h"
#include"MotionProfiledController.h"
#include "driver/pcnt.h"

CustomEncoder *directionEncoder;
MotorController *directionController;
MotorController *translationController;
VelocityControl *directionSpeedControl;
MotionProfiledController *directionMotionProfiling;

double targetSpeed = 0;
double targetDirection = 0;

TaskHandle_t controlLoopTask;

void IRAM_ATTR refreshDirectionEncoderSpeed(){
  directionEncoder->refreshSpeed();
}

void setup() {
  Serial.begin(115200);
  directionEncoder = new CustomEncoder(21, 22, 23.79472, PCNT_UNIT_0);
  directionEncoder->enableSpeedMeasurement(20, 500, refreshDirectionEncoderSpeed);
  directionController = new MotorController(17);
  directionController->setPower(0);
  translationController = new MotorController(15);
  translationController->setPower(0);
  directionMotionProfiling = new MotionProfiledController(directionController, directionEncoder);
  directionMotionProfiling->setPidConstants(0.001,0.01,0.00001,0.05,5);
  directionMotionProfiling->setMotionProfileConstants(1400,-1400,8000,-8000,2);
    xTaskCreatePinnedToCore(
      controlLoop, /* Function to implement the task */
      "controlLoop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &controlLoopTask,  /* Task handle. */
      0); /* Core where the task should run */
}

void loop() {
  readSerialControl();
  delay(5);
}

void controlLoop(void * pvParameters){
  while(true){
    directionMotionProfiling->setPosition(targetDirection);
    Serial.print(" DirectionSpeed:");
    Serial.print(directionEncoder->getSpeed());
    Serial.print(" TargetSpeed:");
    Serial.println(targetSpeed);
    delay(10);
  }
}

void readSerialControl() {
  if (Serial.available() > 0) {
    String incomingValue = Serial.readString();
    String firstValue = incomingValue.substring(0,incomingValue.indexOf(","));
    String secondValue = incomingValue.substring(incomingValue.indexOf(",") + 1, incomingValue.length());
    targetDirection = firstValue.toFloat();
    targetSpeed = secondValue.toFloat();
    if(targetSpeed == 500){
      runControllerCalibration(translationController);
      return;
    } else {
      translationController->setPower(targetSpeed);
    }
  }
}

void runControllerCalibration(MotorController *controller){
  Serial.println("Running Controller Calibration:");
  controller->setPower(0);
  Serial.println("Power is at 0. Hold the calibration button.");
  delay(5000);
  Serial.println("Sweeping PWM...");
  for(int i=1; i<=100; i++){
    float power = ((float)i)/((float)100);
    controller->setPower(power);
    delay(20);
  }
  for(int i=100; i>=-100; i--){
    float power = ((float)i)/((float)100);
    controller->setPower(power);
    delay(20);
  }
  for(int i=-100; i<=100; i++){
    float power = ((float)i)/((float)100);
    controller->setPower(power);
    delay(20);
  }
  for(int i=100; i>=-100; i--){
    float power = ((float)i)/((float)100);
    controller->setPower(power);
    delay(20);
  }
  for(int i=-100; i<=0; i++){
    float power = ((float)i)/((float)100);
    controller->setPower(power);
    delay(20);
  }
  controller->setPower(0);
  delay(1000);
  Serial.println("Calibration complete. Release the calibration button.");
  delay(5000);
}
