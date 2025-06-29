#include "AcceleratorPedalSensors.h"

PedalSensorConfig sensorConfig[NUM_SENSORS] = {
  {A0, 0},
  {A1, 2*sizeof(int)}
};

bool AcceleratorPedalSensors::checkSimilar(float throttleA, float throttleB) {
    return (abs(throttleA - throttleB) <= 10); // Throttle readings must differ by no more than 10 percent
}

void AcceleratorPedalSensors::verifyCalibration() {
    bool overlap = checkSensorRangeOverlap(sensors[0].getSensorRange(), sensors[1].getSensorRange());
    if(sensors[0].isCalibrated() && sensors[1].isCalibrated() && !overlap) calibrated = true;
    if(calibrated) Serial.println("System calibrated correctly!");
}

AcceleratorPedalSensors::AcceleratorPedalSensors() {
    for(int i = 0; i < NUM_SENSORS; i++) {
        sensors[i] = PedalSensor(sensorConfig[i].pin, sensorConfig[i].address);
    }
    verifyCalibration();
}

static bool AcceleratorPedalSensors::checkSensorRangeOverlap(PedalRange sensorRangeA, PedalRange sensorRangeB) {
    return (sensorRangeA.getMin() < sensorRangeB.getMax() && sensorRangeA.getMax() > sensorRangeB.getMin());
}

void AcceleratorPedalSensors::getThrottle(PBCU_CAN_Message *message) {
    if(!calibrated) return;
    
    float throttleValues[NUM_SENSORS];
    for(int i = 0; i < NUM_SENSORS; i++) {
        throttleValues[i] = sensors[i].getThrottle();
        // Serial.println(throttleValues[i]);
        if(throttleValues[i] == -1) return;
    }
    
    if(!checkSimilar(throttleValues[0], throttleValues[1])) return;

    int avgThrottle = (throttleValues[0] + throttleValues[1])/2;
    // Serial.print("Average Throttle: ");
    // Serial.println(avgThrottle);
    message->setThrottle(avgThrottle);
    message->setThrottleFlag(false);
}
