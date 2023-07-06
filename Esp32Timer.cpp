#include"Esp32Timer.h"
#include"esp32-hal-timer.h"

Esp32Timer::Esp32Timer(){
  unsigned char timerNumber = 0;
  while(timerNumber < 4) {
    if(!Esp32Timer::isTimerNumberUsed[timerNumber]){
      Esp32Timer::isTimerNumberUsed[timerNumber] = true;
      break;
    }
    timerNumber++;
  }
  this->baseTimer = timerBegin(timerNumber, 80, true);
}

Esp32Timer::~Esp32Timer(){
  timerEnd(this->baseTimer);
}

void Esp32Timer::enableInterrupt(unsigned long periodInMicrosseconds, void (*callback)()){
  timerAttachInterrupt(this->baseTimer, callback, true);
  timerAlarmWrite(this->baseTimer, periodInMicrosseconds, true);
  timerAlarmEnable(this->baseTimer);
}
