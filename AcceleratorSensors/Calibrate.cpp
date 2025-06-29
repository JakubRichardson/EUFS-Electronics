#include "Calibrate.h"

void setValues(int address, int released, int depressed) {
  int offset = sizeof(int);
  EEPROM.put(address, released);
  EEPROM.put(address + offset, depressed);
}

void waitForEnter() {
  while(true) {
    if(Serial.available() > 0) 
      if(Serial.read() == '\n') return;
  }
}

void runCalibrationSequence() {
  extern struct PedalSensorConfig sensorConfig[2];
  for(int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorConfig[i].pin, INPUT_PULLUP);
  }

  int sensorReadings[NUM_SENSORS][2];
  Serial.println("Release accelerator pedal fully. Press enter to set sensor values: ");
  waitForEnter();
  for(int i = 0; i < NUM_SENSORS; i++) {
    sensorReadings[i][0] = analogRead(sensorConfig[i].pin);
  }
  for(int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Read sensor released value as: ");
    Serial.println(sensorReadings[i][0]);
  }

  Serial.println("Depress accelerator pedal fully. Press enter to set sensor values: ");
  waitForEnter();
  for(int i = 0; i < NUM_SENSORS; i++) {
    // Serial.println(sensorConfig[i].pin);
    sensorReadings[i][1] = analogRead(sensorConfig[i].pin);
  }
  for(int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Read sensor depressed value as: ");
    Serial.println(sensorReadings[i][1]);
  }

  bool valid = true;
  PedalRange sensorRanges[NUM_SENSORS];
  for(int i = 0; i < NUM_SENSORS; i++) {
     sensorRanges[i] = PedalRange(sensorReadings[i][0], sensorReadings[i][1]);
     valid &= sensorRanges[0].isValid();
  }
  Serial.print("Sensor ranges valid: ");
  Serial.println(valid ? "True": "False");

  bool overlap = AcceleratorPedalSensors::checkSensorRangeOverlap(sensorRanges[0], sensorRanges[1]);
  Serial.print("Sensors overlap: ");
  Serial.println(overlap ? "True": "False");
  valid &= !overlap;

  if(valid) {
    for(int i = 0; i < NUM_SENSORS; i++) {
      setValues(sensorConfig[i].address, sensorReadings[i][0], sensorReadings[i][1]);
    }
    Serial.println("Calibration Succesful");
  } else {
    Serial.println("Calibration Failed, try again");
  }
}