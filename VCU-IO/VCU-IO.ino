#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Fsm.h>

// Print Debug
const bool LOG_STATE = true;

// CAN Bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;

// APPS Timeout Timer
const uint32_t APPS_ID = 0x41;    // CAN ID to monitor APPS-Status
const uint32_t BRAKES_ID = 0x44;    // CAN ID to monitor APPS-Status
const unsigned int BRAKE_THRESHOLD = 3; // bar
bool brakesEngagedFlag = false;
const unsigned long TIMEOUT_MS = 200; // APPS Monitor - Timeout
bool appsGood = false;                // APPS flag
unsigned long lastAppsMessageTime = 0;    // Timeout timer

// CAN Messages
const uint32_t Inv_rcv = 0x181;
const uint32_t Inv_snd = 0x201;
const uint32_t VCU_State = 0x3EC;
const int VCUIO2VCU_Buttons = 0x3EB; // 1003

// Pins
const int CAN_BUS_STBY = 6;
int ANG_INP[2] = {14, 15};                         // Analog Inputs
int DIG_INP[8] = {11, 12, 21, 20, 19, 18, 17, 16}; // 24/5V Digital Inputs
int DIG_OUT_5V[8] = {0, 1, 2, 3, 4, 5, 7, 8};      // 5V Digital Outputs
int DIG_OUT_24V[2] = {9, 10};                      // 24V Digital Outputs

// Input signals:
const int TS_ACTIVATION_SIDE = DIG_INP[0];
const int TS_ACTIVATION_DASH = DIG_INP[1];
const int R2D_BUTTON = DIG_INP[2];
const int ASMS_SWITCH = DIG_INP[3];

const int WS_RR_PIN = DIG_INP[4];
const int WS_RL_PIN = DIG_INP[5];

const int TS_ACTIVATION_OUT = DIG_OUT_5V[0];

// Button debounce (active LOW; same timing as R2D)
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;

int dashButtonState;
int dashLastButtonState = LOW;
unsigned long dashLastDebounceTime = 0;

int sideButtonState;
int sideLastButtonState = LOW;
unsigned long sideLastDebounceTime = 0;

const unsigned long DEBOUNCE_DELAY = 20;

// =====================
// STATE VARIABLES
// =====================

bool r2dPressed = false;
bool r2dHeld = false;
bool dashPressed = false;
bool dashHeld = false; // TODO: Check needed
bool sidePressed = false;
bool sideHeld = false;
bool asmsState = false;
bool tsActivation = false;
unsigned long r2dHoldUntilMs = 0;
unsigned long dashHoldUntilMs = 0;
unsigned long sideHoldUntilMs = 0;

// RES receiver: last time we saw TPDO1 (0x191) or we acted on boot / sent NMT (for first-frame timeout)
unsigned long lastResActivityMs = 0;

// RES Messages
const int RES_PDO = 0x191;
bool resReceiving = false;
const unsigned long R2D_HOLD_MS = 400;

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
const unsigned int INV_ENBL_MSG = 64;
const unsigned int INV_DISBL_MSG = 4;
bool invDriveEnabled = false;

unsigned long lastRequestTime = 0;
const unsigned long REQUEST_CYCLE_TIME = 100;
bool invReceivingRun = false;
bool invReceivingEnbl = false;

int lastTxUs = 0;
unsigned long lastButtonsCanTxMs = 0;
int lastButtonsCanData = 0;
bool hasLastButtonsCanData = false;
bool lastR2dHeldDebug = false;
bool lastTsActivationDebug = false;
bool lastAsmsStateDebug = false;
unsigned long lastWheelSerialMs = 0;

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
const int heartRate = 10; // ms
void heartBeat() {
  if (millis() - lastHeartBeatTime < heartRate) return;
  lastHeartBeatTime = millis();
  CAN_message_t msg;
  msg.id = VCU_State;
  msg.buf[0] = CURRENT_STATE;
  msg.buf[1] = 0;
  msg.len = 2;
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

void resRequest() {
  // Request RES mode
  CAN_message_t msg;
  msg.id = 0x000;
  msg.buf[0] = 0x01;
  msg.buf[1] = 0x11;
  msg.len = 2;
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
    appsGood = !(msg.buf[1] & 0x04);
  }
}

void brakesCallback(const CAN_message_t &msg) {
  if (msg.id == BRAKES_ID) {
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

unsigned long lastDebug = false;
void duringTractiveSystemActive() {
  // 1. Read RFE. If goes low go to TS-Off
  if (!RFE) {
    fsm.trigger(TS_DEACTIVATION_EVENT);
  }
  // 2. Read button. If pressed go to Manual Driving
  if (r2dHeld && brakesEngagedFlag) {
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
  enableInverter(true);
  delay(1);
  enableInverter(false);
  delay(1);
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
  if (r2dPressed) {
    Serial.print("Button: ");
    Serial.println(r2dPressed);
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

void sendButtonsCANIfNeeded() {
  if (millis() - lastButtonsCanTxMs > 100) {
    uint8_t data = buildButtonsCanData();
    sendButtonsCAN(data);
    lastButtonsCanTxMs = millis();
  }
}

// Returns debounced active-low level. If edgeToLowOut != nullptr, sets *edgeToLowOut
// true for one call when the stable state transitions to LOW (R2D pulse semantics).
bool updateDebouncedActiveLow(int pin, int &stableState, int &lastReading,
                              unsigned long &lastDebounceMs, bool *edgeToLowOut) {
  if (edgeToLowOut) {
    *edgeToLowOut = false;
  }

  int reading = digitalRead(pin);

  if (reading != lastReading) {
    lastDebounceMs = millis();
  }

  if ((millis() - lastDebounceMs) > DEBOUNCE_DELAY) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW && edgeToLowOut) {
        *edgeToLowOut = true;
      }
    }
  }

  lastReading = reading;
  return stableState == LOW;
}

bool readActiveLowButtonEdge(int pin, int &stableState, int &lastReading,
                             unsigned long &lastDebounceMs) {
  bool edge = false;
  updateDebouncedActiveLow(pin, stableState, lastReading, lastDebounceMs, &edge);
  return edge;
}

bool updateHeldHighFromEdge(bool edgeToHigh, unsigned long holdMs, unsigned long &holdUntilMs) {
  unsigned long nowMs = millis();
  if (edgeToHigh) {
    holdUntilMs = nowMs + holdMs;
    return true;
  }
  return (long)(holdUntilMs - nowMs) > 0;
}

int buildButtonsCanData() {
  int data = 0;
  // ASMS → bit 0
  data |= (asmsState & 0x01) << 0;
  // Combined TS activation output (from dash/side selection logic) → bit 1
  data |= (tsActivation & 0x01) << 1;
  // R2D → bit 2
  data |= (r2dHeld & 0x01) << 2;
  return data;
}

void sendButtonsCAN(uint8_t data) {
  CAN_message_t msg;
  msg.id = VCUIO2VCU_Buttons;
  msg.len = 1;
  msg.buf[0] = data;
  Can0.write(msg);
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
  Can0.setMBFilter(MB2, BRAKES_ID);
  Can0.onReceive(MB2, brakesCallback);
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
    // if(!resReceiving) resRequest();
  }
  heartBeat(); // Send status heartbeat

  // Read inputs
  r2dPressed = 
    readActiveLowButtonEdge(R2D_BUTTON, buttonState, lastButtonState, lastDebounceTime);
  r2dHeld = updateHeldHighFromEdge(r2dPressed, R2D_HOLD_MS, r2dHoldUntilMs);
  dashPressed =
    readActiveLowButtonEdge(TS_ACTIVATION_DASH, dashButtonState, dashLastButtonState,
                            dashLastDebounceTime);
  dashHeld = updateHeldHighFromEdge(dashPressed, R2D_HOLD_MS, dashHoldUntilMs);
  sidePressed =
    readActiveLowButtonEdge(TS_ACTIVATION_SIDE, sideButtonState, sideLastButtonState,
                            sideLastDebounceTime);
  sideHeld = updateHeldHighFromEdge(sidePressed, R2D_HOLD_MS, sideHoldUntilMs);
  asmsState = digitalRead(ASMS_SWITCH);

  // =====================
  // TS Activation Logic (mux; pulse one loop per press, same as R2D-style edges)
  // =====================

  if (!asmsState) {
    tsActivation = dashHeld;
  } else {
    tsActivation = sideHeld;
  }

  // Output
  digitalWrite(TS_ACTIVATION_OUT, tsActivation);

  // =====================
  // CAN TX
  // =====================

  sendButtonsCANIfNeeded();
}
