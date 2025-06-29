#ifndef PedalSensor_h
#define PedalSensor_h

#include <EEPROM.h>
#include "PedalRange.h"
#include "Arduino.h"

class PedalSensor {
  private:
    int pin;
    bool calibrated = false;
    PedalRange pedalSensorRange;
    void loadCalibrationValues(int address);

  public:
    PedalSensor();
    PedalSensor(int pin, int address);
    bool isCalibrated();
    PedalRange getSensorRange();
    float getThrottle();
};

#endif
