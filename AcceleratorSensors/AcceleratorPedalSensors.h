#ifndef AcceleratorPedalSensors_h
#define AcceleratorPedalSensors_h

#include "PedalSensorConfig.h"
#include "PedalSensor.h"
#include "PedalRange.h"
#include "PBCU_CAN_Message.h"
#include "Arduino.h"

#define NUM_SENSORS 2 // Do not change! Changes would have to be made to work with more sensors

class AcceleratorPedalSensors {
  private:
    bool calibrated = false;
    PedalSensor sensors[NUM_SENSORS];

    bool checkSimilar(float throttleA, float throttleB);
    void verifyCalibration();

  public:
    AcceleratorPedalSensors();
    static bool checkSensorRangeOverlap(PedalRange sensorRangeA, PedalRange sensorRangeB);
    void getThrottle(PBCU_CAN_Message *message);
};

#endif
