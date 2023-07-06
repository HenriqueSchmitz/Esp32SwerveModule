#ifndef custom_encoder_h
#define custom_encoder_h

#include"Rotary.h"
#include"Esp32Timer.h"
#include "driver/pcnt.h"

class CustomEncoder {
public:
    CustomEncoder(unsigned char encoderPinA, unsigned char encoderPinB, double ticksPerUnit, pcnt_unit_t counterUnit);
    ~CustomEncoder();
    void enableSpeedMeasurement(unsigned char numberOfSpeedSamples, unsigned int speedSamplingPeriod, void (*speedCallback)());
    void refreshSpeed();
    double getPosition();
    void setPosition(double value);
    double getSpeed();

private:
    void setupCounterPeripheral(unsigned char encoderPinA, unsigned char encoderPinB);
    void storeCounterValue();

    Rotary *rotary;
    pcnt_unit_t counterUnit;
    int64_t ticks;
    double ticksPerUnit;
    double positionMultiplier;

    Esp32Timer *speedInterruptTimer;
    bool isSpeedReadingEnabled;
    unsigned char numberOfSpeedSamples;
    unsigned char speedSavePosition;
    unsigned int speedSamplingPeriod;
    double speedMultiplier;
    int64_t previousTicks;
    int64_t *tickDeltas;

};

#endif
