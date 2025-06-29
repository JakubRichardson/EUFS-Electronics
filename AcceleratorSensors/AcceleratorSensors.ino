#include "PBCU_CAN_Message.h"
#include "AcceleratorPedalSensors.h"
#include "Calibrate.h"

#include <SPI.h>
#include <mcp2515_can.h>

#define CAN_2515
const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN);

// TODO: Add CAN
// TODO: Separate out Calibrate (including private)
// TODO: Improve PedalRange
// TODO: Remove arduino imports
// TODO: Complete brakes

// Note: Array of bytes error

#define Standard_Frame 0
#define APPS2VCU_Front_State 0x41
#define APPS2VCU_Front_Readings 0x45

void sendMessage(int *msg) {
    // unsigned char data[4];
    // int j = 0;
    // for(int i = 0; i < 2; i++) {
    //   Serial.println(msg[i]);
    //   Serial.print("Message: ");

    //   Serial.println("Bytes: ");
    //   byte low = msg[i];
    //   byte high = msg[i] >> 8;
    //   Serial.println(high);
    //   Serial.println(low);
    //   data[j++] = high;
    //   data[j++] = low;
    // }

    // Serial.println("Data: ");
    // for(int i = 0; i < 4; i++) {
    //   Serial.println(data[i]);
    // }

  byte data[3];
  byte low = msg[0];
  byte high = msg[0] >> 8;
  data[0] = low;
  data[1] = high;
  data[2] = msg[1];

  // Serial.print("Sending data: ");
  // for(int i = 0; i < 3; i++) {
  //   Serial.print(data[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");

  CAN.MCP_CAN::sendMsgBuf(APPS2VCU_Front_Readings, Standard_Frame, 3, data);
}

void sendStatus(bool brakeFlag, bool accelFlag) {
  byte status[1] = {0};
  status[0] += 2*accelFlag;
  status[0] += brakeFlag;

  CAN.MCP_CAN::sendMsgBuf(APPS2VCU_Front_State, Standard_Frame, 1, status);
}

class BrakePressureSensor {
  private:
    int pin = A3;

    bool checkValid(int reading) {
      int minVal = 100; // Change This
      int maxVal = 1000; // Change This

      return (reading >= minVal && reading <= maxVal);
    }

  int convertToPressure(int reading) {
    const float sensitivity = (28.57/1000.0); // V/bar'
    const float offset = 0.5; // V
    const float pullUpCorrection = 0.04; // V
    float voltage = reading * (5.0 / 1023.0) - pullUpCorrection;
    return round((voltage - offset) / (sensitivity)); 
  }

  public:
    BrakePressureSensor() {
      pinMode(pin, INPUT_PULLUP);
    }

    int getPressure(PBCU_CAN_Message *message) {
      int pressureReading = analogRead(pin);
      // Serial.print("Pressure sensor value: ");
      // Serial.println(pressureReading);

      if(checkValid(pressureReading)) {
        int brakePressure = convertToPressure(pressureReading);
        // Serial.print("Pressure: ");
        // Serial.println(brakePressure);
        message->setBrakePressure(brakePressure);
        message->setBrakePressureFlag(false);
      }
    }

};

void setup() {
  Serial.begin(9600);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  Serial.println("CAN init ok!");
  
  // runCalibrationSequence();

  AcceleratorPedalSensors pedal;
  BrakePressureSensor pressureSensor;
  while (true) {
    // Create Message
    PBCU_CAN_Message message;

    // Serial.println("Waiting: ");
    // waitForEnter();
    delay(98);
    pedal.getThrottle(&message);
    pressureSensor.getPressure(&message);
    int msg[2];
    message.getMessage(msg);
    bool brakeFlag;
    bool accelFlag;
    message.getStatus(&brakeFlag, &accelFlag);
    sendStatus(brakeFlag, accelFlag);
    sendMessage(msg);
  }
}

void loop() {}