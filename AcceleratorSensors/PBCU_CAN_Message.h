#ifndef PBCU_CAN_Messages_h
#define PBCU_CAN_Messages_h

#include "Arduino.h"

class PBCU_CAN_Message {
  private:
    int brakePressure;
    int throttlePercentage;
    bool invalidBrakePressure;
    bool invalidThrottle;

  public:
    PBCU_CAN_Message();
    void setBrakePressure(float pressure);
    void setThrottle(int throttle);
    void setBrakePressureFlag(bool flag);
    void setThrottleFlag(bool flag);
    void getMessage(int *msg);
    void getStatus(bool *brakeFlag, bool *accelFlag);
};

#endif
