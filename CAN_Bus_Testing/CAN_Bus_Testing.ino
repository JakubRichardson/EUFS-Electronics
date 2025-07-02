#include <FlexCAN_T4.h>

// CAN Bus
const int CAN_BUS_STBY = 6;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;


// CAN IDs
const int brakeID = 0x41; // APPS-Status
const int stateID = 0x54; // VCU-Status
int count = 0;

// Brake Pressure
const int brakePressureThreshold = 2; // bar

// State definitions implemented as enum: B0001 = OFF, B0010 = READY, B0011 = DRIVING, B0100 = EMERGENCY, B0101 = FINISHED, B0110 = MANUAL
typedef enum : uint8_t { 
  OFF = 1,
  READY = 2,
  DRIVING = 3,
  EMERGENCY = 4,
  FINISHED = 5,
  MANUAL = 6
} State;

State CURRENT_STATE = OFF;      // Current state
bool CURRENT_BRAKE_LIGHT_STATE = false;
bool readyTrigger = false;
bool emergencyTrigger = false;

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void updateBrakeLight(const CAN_message_t &msg) {
  if (msg.id == brakeID) {
    uint16_t frontPressure = msg.buf[0] | (msg.buf[1] << 8);
    Serial.println(frontPressure);
  }
}

void updateState(const CAN_message_t &msg) {
  if (msg.id == stateID) {
    State NEW_STATE = static_cast<State>(msg.buf[0] & 0x0F);
    if (NEW_STATE != CURRENT_STATE) {
      CURRENT_STATE = NEW_STATE;
      Serial.println(NEW_STATE);
    }
  }
}

void setup() {
  // Initialise Serial Interface
  Serial.begin(9600);

  // Initialise CAN
  pinMode(CAN_BUS_STBY, OUTPUT); digitalWrite(CAN_BUS_STBY, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
  Can0.setMBFilter(MB0, brakeID);
  Can0.onReceive(MB0, updateBrakeLight);
  Can0.setMBFilter(MB1, stateID);
  Can0.onReceive(MB1, updateState);
  Can0.mailboxStatus();
}

void loop() {

}
