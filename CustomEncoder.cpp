#include"CustomEncoder.h"
#include"Esp32Timer.h"
#include "driver/pcnt.h"

CustomEncoder::CustomEncoder(unsigned char encoderPinA, unsigned char encoderPinB, double ticksPerUnit, pcnt_unit_t counterUnit){
  this->counterUnit = counterUnit;
  this->setupCounterPeripheral(encoderPinA, encoderPinB);
  this->encoderPinA = encoderPinA;
  this->ticks = 0;
  this->previousTicks = 0;
  if(ticksPerUnit == 0) {
    ticksPerUnit = 1;
  }
  this->ticksPerUnit = ticksPerUnit;
  this->positionMultiplier = 1 / this->ticksPerUnit;
  this->isSpeedReadingEnabled = false;
  if(numberOfSpeedSamples == 0) {
    numberOfSpeedSamples = 1;
  }
  this->numberOfSpeedSamples = numberOfSpeedSamples;
  this->speedSavePosition = 0;
  this->tickDeltas = new int64_t[numberOfSpeedSamples];
  for(int speedSamplePosition = 0; speedSamplePosition < numberOfSpeedSamples; speedSamplePosition++){
    this->tickDeltas[speedSamplePosition] = 0;
  }
}

CustomEncoder::~CustomEncoder(){
  if(this->isSpeedReadingEnabled) {
    delete this->tickDeltas;
  }
}

void CustomEncoder::setupCounterPeripheral(unsigned char encoderPinA, unsigned char encoderPinB){
  pcnt_config_t pcnt_config = { };
  pcnt_config.pulse_gpio_num = encoderPinA;
  pcnt_config.ctrl_gpio_num = encoderPinB;
  pcnt_config.unit = this->counterUnit;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.counter_h_lim = INT16_MAX;
  pcnt_config.counter_l_lim = INT16_MIN;
  pcnt_config.pos_mode = PCNT_COUNT_DIS;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_unit_config(&pcnt_config);
  
  pcnt_set_filter_value(this->counterUnit, 250);  // Filter Runt Pulses
  pcnt_filter_enable(this->counterUnit);
  
  pcnt_counter_pause(this->counterUnit); // Initial PCNT init
  pcnt_counter_clear(this->counterUnit);
  pcnt_counter_resume(this->counterUnit);
}

void CustomEncoder::storeCounterValue(){
  int16_t pcntValue;
  pcnt_counter_pause(this->counterUnit);
  pcnt_get_counter_value(this->counterUnit, &pcntValue);
  pcnt_counter_clear(this->counterUnit);
  pcnt_counter_resume(this->counterUnit);
  this->ticks += pcntValue;
}

void CustomEncoder::enableSpeedMeasurement(unsigned char numberOfSpeedSamples, unsigned int speedSamplingPeriod, void (*speedCallback)()){
  this->speedSavePosition = 0;
  this->numberOfSpeedSamples = numberOfSpeedSamples;
  this->speedSamplingPeriod = speedSamplingPeriod;
  this->tickDeltas = new int64_t[numberOfSpeedSamples];
  for(int speedSamplePosition = 0; speedSamplePosition < numberOfSpeedSamples; speedSamplePosition++){
    this->tickDeltas[speedSamplePosition] = 0;
  }
  this->speedInterruptTimer = new Esp32Timer();
  this->speedInterruptTimer->enableInterrupt(speedSamplingPeriod, speedCallback);
  this->speedMultiplier = ((double)1000000) / (((double)this->numberOfSpeedSamples) * ((double)this->speedSamplingPeriod) * this->ticksPerUnit);
  this->isSpeedReadingEnabled = true;
}

void IRAM_ATTR CustomEncoder::refreshSpeed(){
  this->storeCounterValue();
  int64_t ticksDelta = this->ticks - this->previousTicks;
  this->tickDeltas[this->speedSavePosition] = ticksDelta;
  this->speedSavePosition++;
  if(this->speedSavePosition >= this->numberOfSpeedSamples){
    this->speedSavePosition = 0;
  }
  this->previousTicks = this->ticks;
}

double CustomEncoder::getPosition(){
  this->storeCounterValue();
  return this->ticks/this->ticksPerUnit;
}

void CustomEncoder::setPosition(double value){
  this->ticks = (int64_t)(value * this->ticksPerUnit);
}

double CustomEncoder::getSpeed(){
  if(this->isSpeedReadingEnabled) {
    int64_t totalTicksDelta = 0;
    for(int measurement = 0; measurement < this->numberOfSpeedSamples; measurement++){
      totalTicksDelta += this->tickDeltas[measurement];
    }
    return (((double)totalTicksDelta) * this->speedMultiplier);
  } else {
    return 0;
  }
}
