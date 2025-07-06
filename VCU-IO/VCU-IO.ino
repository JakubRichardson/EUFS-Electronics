#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Fsm.h>

// TODO - Tests:
// Check cyclic receive from inverter
// Check enable/disable works

// Print Debug
const bool LOG_STATE = true;

// CAN Bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;

// APPS Timeout Timer
const uint32_t APPS_ID = 0x41;    // CAN ID to monitor APPS-Status
const unsigned int BRAKE_THRESHOLD = 6; // bar
bool brakesEngagedFlag = false;
const unsigned long TIMEOUT_MS = 100; // APPS Monitor - Timeout
bool appsGood = false;                // APPS flag
unsigned long lastAppsMessageTime = 0;    // Timeout timer

// CAN Messages
const uint32_t Inv_rcv = 0x181;
const uint32_t Inv_snd = 0x201;
const uint32_t VCU_State = 0x3;

// Pins
const int CAN_BUS_STBY = 6;
int ANG_INP[2] = {14, 15};                         // Analog Inputs
int DIG_INP[8] = {11, 12, 21, 20, 19, 18, 17, 16}; // 24/5V Digital Inputs
int DIG_OUT_5V[8] = {0, 1, 2, 3, 4, 5, 7, 8};      // 5V Digital Outputs
int DIG_OUT_24V[2] = {9, 10};                      // 24V Digital Outputs

// Input signals:
const int R2D_BUTTON = DIG_INP[0]; // R2D button input pin
const int TSMS = DIG_INP[4];       // TSMS - 24V input
int buttonState;                   // current button reading
int lastButtonState = LOW;         // previous button reading
// Timers:
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;      // the last time the output pin was toggled
const unsigned long DEBOUNCE_DELAY = 20; // the debounce time; increase if the output flickers

// Inverter RFE Message
bool RFE = false;
const int RUN_ID = 0xE8; // TODO
const int READ_ID = 0x3D;
const int GO_drive_enable = 0xE3;
const int RDY = 0xE2;

// Inverter Enable Message
unsigned long lastEnableSendTime = 0;        // the last time enable was sent to the inverter
const unsigned long ENABLE_CYCLE_TIME = 100; // time (ms) between sending enable messages
const int INV_ENBL_ID = 0x51;                // TODO
const unsigned int INV_ENBL_MSG = 8;
const unsigned int INV_DISBL_MSG = 12;
bool invDriveEnabled = false;

unsigned long lastRequestTime = 0;
const unsigned long REQUEST_CYCLE_TIME = 100;
bool invReceivingRun = false;
bool invReceivingEnbl = false;

// State definitions implemented as enum: B000 = OFF, B001 = ACTIVE, B010 = MANUAL, B111 = CORSA
typedef enum : uint8_t { 
  TS_OFF = 1,
  TS_ACTIVE = 2,
  MANUAL = 3,
  CORSA = 7
} VCU_State_Enum;

VCU_State_Enum CURRENT_STATE = TS_OFF;      // Current state

// Events
#define TS_ACTIVATION_EVENT 0
#define TS_DEACTIVATION_EVENT 1
#define R2D_BUTTON_EVENT 2

// FSM States
State stateTractiveSystemOff(&onEnterTractiveSystemOff, &duringTractiveSystemOff, NULL);
State stateTractiveSystemActive(&onEnterTractiveSystemActive, &duringTractiveSystemActive, NULL);
State stateManualDriving(&onEnterManualDriving, &duringManualDriving, NULL);
Fsm fsm(&stateTractiveSystemOff);

// Current State Heartbeat
unsigned long lastHeartBeatTime = 0;
const int heartRate = 100; // ms
void heartBeat() {
  if (millis() - lastHeartBeatTime < heartRate) return;
  lastHeartBeatTime = millis();
  CAN_message_t msg;
  msg.id = VCU_State;
  msg.buf[0] = CURRENT_STATE;
  msg.len = 1;
  Can0.write(msg);
}

void inverterRequest(int request) {
  // Request RUN flag
  CAN_message_t msg;
  msg.id = Inv_snd;
  msg.buf[0] = READ_ID;
  msg.buf[1] = request;
  msg.buf[2] = 0x64; // cyclic at 100ms
  msg.len = 3;
  Can0.write(msg);
}

void checkFlag() {
  if (millis() - lastAppsMessageTime < TIMEOUT_MS) return;
  appsGood = false;
  if (LOG_STATE) {
    Serial.println("APPS Timeout");
  }
}

void appsCallback(const CAN_message_t &msg) {
  if (msg.id == APPS_ID) {
    lastAppsMessageTime = millis();
    appsGood = !(msg.buf[5] & 0x04);
    unsigned int frontBrakePressure = ((msg.buf[1] << 8) | msg.buf[0]);
    brakesEngagedFlag = (frontBrakePressure >= BRAKE_THRESHOLD);
  }
}

void handleReceive(const CAN_message_t &msg) {
  if (msg.id == Inv_rcv) {
    if(msg.buf[0] == RUN_ID){
      if (!invReceivingRun) {
        invReceivingRun = true;
      }
      RFE = msg.buf[1] & 0x01;
    }
    else if (msg.buf[0] == INV_ENBL_ID){
      if (!invReceivingEnbl) {
        invReceivingEnbl = true;
      }
      if(msg.buf[1] == INV_ENBL_MSG) {
        invDriveEnabled = true;
      } else {
        invDriveEnabled = false;
      }
    }
  }
}

/**
 * @brief Send inverter enable CAN message
 *
 * @param bool enable - send enable/disable
 */
void enableInverter(bool enable) {
  if ((invDriveEnabled != enable) && ((millis() - lastEnableSendTime) > ENABLE_CYCLE_TIME)) {
    lastEnableSendTime = millis();
    CAN_message_t msg;
    msg.id = Inv_snd;
    msg.buf[0] = INV_ENBL_ID;
    if (enable) {
      msg.buf[1] = INV_ENBL_MSG;
    } else {
      msg.buf[1] = INV_DISBL_MSG;
    }
    msg.len = 3;
    Can0.write(msg);
  }
}

/**
 * @brief Reads button to check if pressed
 *
 * @return bool - button pressed
 */
bool buttonPressed() {
  bool pressed = false;
  // Handle R2D Button
  int buttonReading = digitalRead(R2D_BUTTON);
  if (buttonReading != lastButtonState) {
    lastDebounceTime = millis(); // Reset debounce timer
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (buttonReading != buttonState) {
      buttonState = buttonReading;
      if (buttonState == LOW){ // Button Released
        pressed = true;
      }
    }
  }
  lastButtonState = buttonReading;
  return pressed;
}

void onEnterTractiveSystemOff() {
  if (LOG_STATE) {
    Serial.println("TS-Off");
  }
  CURRENT_STATE = TS_OFF;
}

void duringTractiveSystemOff() {
  // 1. Read RFE. If RFE goes high go to TS-Active
  if (RFE) {
    fsm.trigger(TS_ACTIVATION_EVENT);
  }
  // 2. Send Inverter Disable
  enableInverter(false);
}

void onEnterTractiveSystemActive() {
  if (LOG_STATE) {
    Serial.println("TS Active");
  }
  CURRENT_STATE = TS_ACTIVE;
}

void duringTractiveSystemActive() {
  // 1. Read RFE. If goes low go to TS-Off
  if (!RFE) {
    fsm.trigger(TS_DEACTIVATION_EVENT);
  }
  // 2. Read button. If pressed go to Manual Driving
  bool pressed = buttonPressed();
  if (pressed && brakesEngagedFlag) {
    fsm.trigger(R2D_BUTTON_EVENT);
  }
  // 3. Send Inverter Disable
  enableInverter(false);
}

void onEnterManualDriving() {
  if (LOG_STATE) {
    Serial.println("Manual Driving");
  }
  CURRENT_STATE = MANUAL;
  lastAppsMessageTime = millis();
}

void duringManualDriving() {
  // 1. Read RFE. If goes low go to TS-Off
  if (!RFE) {
    Serial.print("RFE: ");
    Serial.println(RFE);
    // Don't need to disable Inverter as RFE low.
    // Will need to go back through TS-active state again to enable again
    fsm.trigger(TS_DEACTIVATION_EVENT);
  }
  // 2. Read button. If pressed go to TS-Active
  bool pressed = buttonPressed();
  if (pressed) {
    Serial.print("Button: ");
    Serial.println(pressed);
    fsm.trigger(R2D_BUTTON_EVENT);
  }
  // 3. Check APPS flags. If implausible or not receiving -> deactivate drive
  checkFlag(); // Check APPS flag
  if (!appsGood) {
    Serial.print("appsGood: ");
    Serial.println(appsGood);
    fsm.trigger(R2D_BUTTON_EVENT);
  }
  // 4. Send Inverter Enable
  enableInverter(true);
}

void setupFSM() {
  // State: Tractive System Off
  fsm.add_transition(&stateTractiveSystemOff, &stateTractiveSystemActive, TS_ACTIVATION_EVENT, NULL);

  // State: Tractive System Active
  fsm.add_transition(&stateTractiveSystemActive, &stateManualDriving, R2D_BUTTON_EVENT, NULL);
  fsm.add_transition(&stateTractiveSystemActive, &stateTractiveSystemOff, TS_DEACTIVATION_EVENT, NULL);

  // State: Manual Driving
  fsm.add_transition(&stateManualDriving, &stateTractiveSystemActive, R2D_BUTTON_EVENT, NULL);
  fsm.add_transition(&stateManualDriving, &stateTractiveSystemOff, TS_DEACTIVATION_EVENT, NULL);
}

void setupInputs(int *pins, int size) {
  for (int i = 0; i < size; i++) {
    pinMode(pins[i], INPUT);
  }
}

void setupOutputs(int *pins, int size) {
  for (int i = 0; i < size; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void setup() {
  // Initialise Serial Interface
  Serial.begin(9600);

  // Initilise Pins
  // setupInputs(ANG_INP, 2); // Analog Inputs
  setupInputs(DIG_INP, 8);      // Digital Inputs
  setupOutputs(DIG_OUT_5V, 8);  // 5V Digital Outputs
  setupOutputs(DIG_OUT_24V, 2); // 24V Digital Outputs

  // Initialise CAN
  pinMode(CAN_BUS_STBY, OUTPUT);
  digitalWrite(CAN_BUS_STBY, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(16);
  Can0.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
  Can0.setMBFilter(MB0, APPS_ID);
  Can0.onReceive(MB0, appsCallback);
  Can0.setMBFilter(MB1, Inv_rcv);
  Can0.onReceive(MB1, handleReceive);
  Can0.mailboxStatus();

  // Setup State Machine
  setupFSM();
  onEnterTractiveSystemOff();
}

void loop() {
  fsm.run_machine();
  if ((millis() - lastRequestTime) > REQUEST_CYCLE_TIME) {
    lastRequestTime = millis();
    if(!invReceivingRun) inverterRequest(RUN_ID);
    if(!invReceivingEnbl) inverterRequest(INV_ENBL_ID);
  }
  heartBeat(); // Send status heartbeat
}
