#ifndef PedalRange_h
#define PedalRange_h

#include "Arduino.h"

class PedalRange {
  private:
    bool valid = false;
    bool invert;
    int range_max = -1;
    int range_min = -1;

    void setValid();
    void setBounds();

  public:
    PedalRange();
    PedalRange(int released, int depressed);
    int getMin();
    int getMax();
    bool isValid();
    int clampReading(int reading);
    bool checkValidReading(int reading);
    float mapToThrottle(int reading);
};

#endif
