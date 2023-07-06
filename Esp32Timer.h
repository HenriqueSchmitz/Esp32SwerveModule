#ifndef esp_32_timer_h
#define esp_32_timer_h

#include"esp32-hal-timer.h"

class Esp32Timer {
public:
    Esp32Timer();
    ~Esp32Timer();
    void enableInterrupt(unsigned long periodInMicrosseconds, void (*callback)());

private:
  inline static bool isTimerNumberUsed[4] = {false, false, false, false};
  hw_timer_t *baseTimer;

};
//
//Esp32Timer::isTimerNumberUsed[0] = false;
//Esp32Timer::isTimerNumberUsed[1] = false;
//Esp32Timer::isTimerNumberUsed[2] = false;
//Esp32Timer::isTimerNumberUsed[3] = false;

#endif
