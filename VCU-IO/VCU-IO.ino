#include <Arduino.h>
#include <FlexCAN_T4.h>

// Wheelspeed struct
struct WheelState {
  volatile unsigned long lastEdgeUs = 0;
  volatile unsigned long lastIntervalUs = 0;
  volatile unsigned long lastValidPulseUs = 0;
  volatile unsigned int rpm_x10 = 0;
};

// Inverter/RES messages
unsigned long lastRequestTime = 0;
const unsigned long REQUEST_CYCLE_TIME = 100;

// Inverter Messages
const uint32_t Inv_rcv = 0x181;
const uint32_t Inv_snd = 0x201;
const int RUN_ID = 0xE8;
const int READ_ID = 0x3D;
const int INV_ENBL_ID = 0x51;
bool invReceivingRun = false;
bool invReceivingEnbl = false;

// CANopen RES receiver (DE7.4.8): Node-ID fixed 0x11, 500 kbit/s
const int RES_NODE_ID = 0x11;
const int CAN_NMT_BOOTUP = 0x700 + RES_NODE_ID;   // 0x711, boot-up (data 0x00)
const int CAN_NMT_CMD = 0x000;                    // NMT master -> slaves
const int CAN_TPDO1_STATUS = 0x180 + RES_NODE_ID; // 0x191, PDOs 2000-2007 @ 30 ms
bool resReceiving = false;

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int CAN_BAUDRATE = 500000;

// CAN IDs (application)
const int VCUIO2VCU_Buttons = 0x3EB; // 1003

// Pins
const int CAN_BUS_STBY = 6;
// Pin arrays (kept as requested)
int ANG_INP[2] = {14, 15};
int DIG_INP[8] = {11, 12, 21, 20, 19, 18, 17, 16};
int DIG_OUT_5V[8] = {0, 1, 2, 3, 4, 5, 7, 8};
int DIG_OUT_24V[2] = {9, 10};

// Assign signals using array indices
const int TS_ACTIVATION_SIDE = DIG_INP[0];
const int TS_ACTIVATION_DASH = DIG_INP[1];
const int TS_ACTIVATION_OUT = DIG_OUT_5V[0];
const int R2D_BUTTON = DIG_INP[2];
const int ASMS_SWITCH = DIG_INP[3];
const int WS_RR_PIN = DIG_INP[4];
const int WS_RL_PIN = DIG_INP[5];

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
const unsigned long R2D_HOLD_MS = 400;

// =====================
// STATE VARIABLES
// =====================

bool r2dPressed = false;
bool r2dHeld = false;
bool dashPressed = false;
bool dashHeld = false;
bool sidePressed = false;
bool sideHeld = false;
bool asmsState = false;
bool tsActivation = false;
unsigned long r2dHoldUntilMs = 0;
unsigned long dashHoldUntilMs = 0;
unsigned long sideHoldUntilMs = 0;
unsigned long lastButtonsCanTxMs = 0;

// =====================
// FUNCTIONS
// =====================

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

// =====================
// CAN Receive
// =====================

void handleReceive(const CAN_message_t &msg) {
  Serial.println("Received");
  if (msg.id == Inv_rcv) {
    if(msg.buf[0] == RUN_ID){
      if (!invReceivingRun) invReceivingRun = true;
    } else if (msg.buf[0] == INV_ENBL_ID){
      if (!invReceivingEnbl) invReceivingEnbl = true;
    }
  }
}

void handleRES(const CAN_message_t &msg) {
  if (msg.id == CAN_TPDO1_STATUS) {
    if (!resReceiving) {
      resReceiving = true;
    }
  }
}

void inverterRequest(int request) {
  // Request RUN flag and KERN mode
  CAN_message_t msg;
  msg.id = Inv_snd;
  msg.buf[0] = READ_ID;
  msg.buf[1] = request;
  msg.buf[2] = 0x64; // cyclic at 100ms
  msg.len = 3;
  Can0.write(msg);
}

// =====================
// RES Startup
// =====================

// NMT: start node 0x11 (operational). CAN-ID 0x000, data 0x01 0x11
void resRequest() {
  // Request RES mode
  CAN_message_t msg;
  msg.id = CAN_NMT_CMD;
  msg.buf[0] = 0x01;
  msg.buf[1] = RES_NODE_ID;
  msg.len = 2;
  Can0.write(msg);
}

// =====================
// Buttons
// =====================

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

void sendButtonsCAN() {
  uint8_t data = buildButtonsCanData();

  CAN_message_t msg;
  msg.id = VCUIO2VCU_Buttons;
  msg.len = 1;
  msg.buf[0] = data;
  Can0.write(msg);
}

// =====================
// Wheel speeds
// =====================

const int PULSES_PER_REV = 12;
const int MAX_WHEEL_RPM = 2500;
const int STOP_TIMEOUT_US = 200000;
const unsigned long MIN_PULSE_SPACING_US =
  60000000UL / (PULSES_PER_REV * MAX_WHEEL_RPM);

WheelState rlState;
WheelState rrState;

static inline void capturePulse(volatile WheelState &w) {
  int nowUs = micros();
  if (w.lastEdgeUs == 0) {
    w.lastEdgeUs = nowUs;
    w.lastValidPulseUs = nowUs;
    return;
  }

  unsigned long dtUs = nowUs - w.lastEdgeUs;
  if (dtUs < MIN_PULSE_SPACING_US) return;
  w.lastIntervalUs = dtUs;
  w.lastEdgeUs = nowUs;
  w.lastValidPulseUs = nowUs;
}

void rlISR() { capturePulse(rlState); }
void rrISR() { capturePulse(rrState); }

static inline void updateWheelSpeed(volatile WheelState &w) {
  unsigned long intervalUs;
  unsigned long lastValidPulseUs;
  // Atomic copy of interrupt-updated values
  noInterrupts();
  intervalUs = w.lastIntervalUs;
  lastValidPulseUs = w.lastValidPulseUs;
  interrupts();

  unsigned long nowUs = micros();
  unsigned int rpm_x10_calc = 0;

  if (lastValidPulseUs == 0 || (nowUs - lastValidPulseUs) > STOP_TIMEOUT_US) {
    rpm_x10_calc = 0;
  } else if (intervalUs != 0) {
    rpm_x10_calc = 
      (unsigned int)(600000000UL / 
      ((unsigned long)PULSES_PER_REV * intervalUs));

    if (rpm_x10_calc > 65535) rpm_x10_calc = 65535;
  }

  // Atomic update
  noInterrupts();
  w.rpm_x10 = rpm_x10_calc;
  interrupts();
}

// =====================
// SETUP
// =====================

void setup() {
  Serial.begin(9600);

  // Setup IO using arrays
  setupInputs(DIG_INP, 8);
  setupOutputs(DIG_OUT_5V, 8);
  setupOutputs(DIG_OUT_24V, 2);

  // Initialise CAN
  pinMode(CAN_BUS_STBY, OUTPUT);
  digitalWrite(CAN_BUS_STBY, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(CAN_BAUDRATE);
  Can0.setMaxMB(16);
  Can0.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
  Can0.setMBFilter(MB0, Inv_rcv);
  Can0.onReceive(MB0, handleReceive);
  Can0.setMBFilter(MB1, CAN_TPDO1_STATUS);
  Can0.onReceive(MB1, handleRES);
  Can0.mailboxStatus();

  // Setup Wheelspeeds
  pinMode(WS_RL_PIN, INPUT);
  pinMode(WS_RR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WS_RL_PIN), rlISR, RISING);
  attachInterrupt(digitalPinToInterrupt(WS_RR_PIN), rrISR, RISING);
}

// =====================
// LOOP
// =====================

void loop() {
  if ((millis() - lastRequestTime) > REQUEST_CYCLE_TIME) {
    lastRequestTime = millis();
    if(!invReceivingRun) inverterRequest(RUN_ID);
    if(!invReceivingEnbl) inverterRequest(INV_ENBL_ID);
    if(!resReceiving) resRequest();
  }

  // Read buttons
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

  // Wheelspeed
  updateWheelSpeed(rlState);
  updateWheelSpeed(rrState);

  // =====================
  // CAN TX
  // =====================
  if (millis() - lastButtonsCanTxMs > 100) {
    lastButtonsCanTxMs = millis();
    sendButtonsCAN();
  }
}
