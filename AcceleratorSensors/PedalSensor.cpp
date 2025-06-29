#include "PedalSensor.h"

// TODO: Move to Seperate File
void PedalSensor::loadCalibrationValues(int address) {
    int offset = sizeof(int);
    int released = -1;
    int depressed = -1;
    EEPROM.get(address, released);
    EEPROM.get(address + offset, depressed);

    PedalRange pedalRange(released, depressed);
    if(pedalRange.isValid()) {
        pedalSensorRange = pedalRange;
        calibrated = true;
        Serial.println("Sensor Calibrated!");
    }
}

PedalSensor::PedalSensor() {}

PedalSensor::PedalSensor(int pin, int address) {
    pinMode(pin, INPUT_PULLUP);
    this->pin = pin;
    loadCalibrationValues(address);
}

bool PedalSensor::isCalibrated() {
    return calibrated;
}

PedalRange PedalSensor::getSensorRange() {
    return pedalSensorRange;
}

float PedalSensor::getThrottle() {
    int sensorReading = pedalSensorRange.clampReading(analogRead(pin));
    if(!pedalSensorRange.checkValidReading(sensorReading)) {
        // Serial.println("Invalid");
        return -1;
    }

    float throttlePercentage = pedalSensorRange.mapToThrottle(sensorReading);
    // Serial.print("Throttle: ");
    // Serial.println(throttlePercentage);
    return throttlePercentage;
}
