#include <FlexCAN_T4.h>

// CAN Bus
const int CAN_BUS_STBY = 6;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;

const int MSG_ID = 0x50;
int count = 0;

void handleReceive(const CAN_message_t &msg) {
  count++;
  if (msg.id == MSG_ID) {
    Serial.print("Received: ");
    Serial.println(count);
  }
}

void setup() {
  // Initialise Serial Interface
  Serial.begin(9600);

  // Initialise CAN
  pinMode(CAN_BUS_STBY, OUTPUT); digitalWrite(CAN_BUS_STBY, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(16);
  // Can0.enableFIFO();
  // Can0.enableFIFOInterrupt();
  Can0.setMB(MB0, RX);
  Can0.setMBFilter(MB0, 0x50);
  Can0.onReceive(MB0, handleReceive);
  Can0.mailboxStatus();
}

void loop() {

}
