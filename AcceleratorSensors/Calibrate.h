#ifndef Calibrate_h
#define Calibrate_h

#include <EEPROM.h>
#include "PedalRange.h"
#include "AcceleratorPedalSensors.h"

void setValues(int address, int released, int depressed);
void waitForEnter();
// void getReleased(int **sensorReadings);
// void getDepressed(int **sensorReadings);
void runCalibrationSequence();

#endif
