#include "PBCU_CAN_Message.h"

PBCU_CAN_Message::PBCU_CAN_Message() {
  brakePressure = 0;
  throttlePercentage = 0;
  invalidBrakePressure = true;
  invalidThrottle = true;
}

void PBCU_CAN_Message::setBrakePressure(float pressure) {
  brakePressure = pressure;
}

void PBCU_CAN_Message::setThrottle(int throttle) {
  throttlePercentage = throttle;
}

void PBCU_CAN_Message::setBrakePressureFlag(bool flag) {
  invalidBrakePressure = flag;
}

void PBCU_CAN_Message::setThrottleFlag(bool flag) {
  invalidThrottle = flag;
}

void PBCU_CAN_Message::getMessage(int *msg) {
  Serial.println("CAN message: ");
  Serial.print("Brake Pressure: ");
  Serial.println(brakePressure);
  Serial.print("Throttle: ");
  Serial.println(throttlePercentage);

  msg[0] = brakePressure;
  msg[1] = throttlePercentage;
}

void PBCU_CAN_Message::getStatus(bool *brakeFlag, bool *accelFlag) {
  Serial.println("CAN status: ");
  Serial.print("Brake flag: ");
  Serial.println(invalidBrakePressure);
  Serial.print("Throttle flag: ");
  Serial.println(invalidThrottle);

  *brakeFlag = invalidBrakePressure;
  *accelFlag = invalidThrottle; 
}
