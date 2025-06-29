#include "PedalRange.h"

#define maxSensorOutput 920
#define minSensorOutput 120
#define emfAllowance 5
#define maxValidReading (maxSensorOutput - emfAllowance)
#define minValidReading (minSensorOutput + emfAllowance)

void PedalRange::setValid() {
    valid = (range_min < range_max) && (range_min > minValidReading) && (range_max < maxValidReading);
}

void PedalRange::setBounds() {
    int maxEMF = range_max + emfAllowance;
    int minEMF = range_min - emfAllowance;
    

    int sensorRange = range_max - range_min + 1;
    int deadzone = sensorRange * 0.07;
    int live_min = range_min + deadzone;
    int live_max = range_max - deadzone;

    Serial.print("Max valid: ");
    Serial.println(maxValidReading);
    Serial.print("Top val: ");
    Serial.println(maxEMF);
    Serial.print("Range max: ");
    Serial.println(range_max);
    Serial.print("Sensor Range: ");
    Serial.println(range_max - range_min + 1);
    Serial.print("Top Deadzone: ");
    Serial.println(live_max);
    Serial.print("Bottom Deadzone: ");
    Serial.println(live_min);
    Serial.print("Range min: ");
    Serial.println(range_min);
    Serial.print("Bottom val: ");
    Serial.println(minEMF);
    Serial.print("Min valid: ");
    Serial.println(minValidReading);
    Serial.println(invert ? "Invert" : "No invert");
}

PedalRange::PedalRange() {}

PedalRange::PedalRange(int released, int depressed) {
    if(released < depressed) {
    range_min = released;
    range_max = depressed;
    invert = false;
    } else {
    range_max = released;
    range_min = depressed;
    invert = true;
    }
    setValid();
    // setBounds();
}

int PedalRange::getMin() {
    return range_min;
}

int PedalRange::getMax() {
    return range_max;
}

bool PedalRange::isValid() {
    return valid;
}

int PedalRange::clampReading(int reading) {
    // Serial.print("Acceleration sensor value: ");
    // Serial.println(sensorReading);
    if(reading < range_min && reading >= range_min - emfAllowance) return range_min;
    if(reading > range_max && reading <= range_max + emfAllowance) return range_max;
    return reading;
}

bool PedalRange::checkValidReading(int reading) {
    // Serial.println(reading);
    return (reading >= range_min && reading <= range_max);
}

float PedalRange::mapToThrottle(int reading) {
    int sensorRange = range_max - range_min + 1;
    int deadzone = sensorRange * 0.07;
    int live_min = range_min + deadzone;
    int live_max = range_max - deadzone;

    float throttlePercentage;
    if (reading < live_min) {
      throttlePercentage = 0;
    } else if (reading > live_max) {
      throttlePercentage = 100;
    } else {
      throttlePercentage = 100 * (reading - live_min)/(live_max - live_min + 1);
    }
    if(invert) throttlePercentage = 100 - throttlePercentage;

    // Serial.print("Sensor Range: ");
    // Serial.println(sensorRange);
    // Serial.print("Deadzone: ");
    // Serial.println(deadzone);
    return throttlePercentage;
}
