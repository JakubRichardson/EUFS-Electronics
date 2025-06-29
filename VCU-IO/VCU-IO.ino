#include <Arduino.h>
#include <Fsm.h>
#include <FlexCAN_T4.h>

// Print Debug
const bool LOG_STATE = true;

// CAN Bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;

// Pins
const int CAN_BUS_STBY = 6;
int ANG_INP[2] = {14, 15}; // Analog Inputs
int DIG_INP[8] = {11, 12, 21, 20, 19, 18, 17, 16}; // 24/5V Digital Inputs
int DIG_OUT_5V[8] = {0, 1, 2, 3, 4, 5, 7, 8}; // 5V Digital Outputs
int DIG_OUT_24V[2] = {9, 10}; // 24V Digital Outputs

// Input signals:
const int R2D_BUTTON = DIG_INP[1];  // R2D button input pin
const int TSMS = DIG_INP[4]; // TSMS - 24V input
int buttonState; // current button reading
int lastButtonState = LOW;  // previous button reading
// Timers:
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
const unsigned long DEBOUNCE_DELAY = 20;   // the debounce time; increase if the output flickers

// Inverter RFE Message
bool RFE = false;
const int RFE_ID = 0x01; // TODO

// Inverter Enable Message
unsigned long lastEnableSendTime = 0;  // the last time enable was sent to the inverter
const unsigned long ENABLE_CYCLE_TIME = 100;   // time (ms) between sending enable messages
const int INV_ENBL_ID = 0x02; // TODO

// Events
#define TS_ACTIVATION_EVENT  0
#define TS_DEACTIVATION_EVENT  1
#define R2D_BUTTON_EVENT  2

// FSM States
State stateTractiveSystemOff(&onEnterTractiveSystemOff, NULL, NULL);
State stateTractiveSystemActive(&onEnterTractiveSystemActive, NULL, NULL);
State stateManualDriving(&onEnterManualDriving, NULL, NULL);
Fsm fsm(&stateTractiveSystemOff);


void handleReceive(const CAN_message_t &msg) {
  if (msg.id == RFE_ID) {
    readRFE(msg);
  }
}

/**
 * @brief Read the value of RFE from the Inverter
 */
void readRFE(const CAN_message_t &msg) {
  // TODO: Complete
}

/**
 * @brief Send inverter enable CAN message
 * 
 * @param bool enable - send enable/disable
 */
void enableInverter(bool enable) {
  if ((millis() - lastEnableSendTime) > ENABLE_CYCLE_TIME) {
    lastEnableSendTime = millis();
    // TODO: Complete
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
      if (buttonState == LOW) { // Button Released
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
  enableInverter(false);
}

void duringTractiveSystemOff() {
  // 1. Read RFE. If RFE goes high go to TS-Active
  if (RFE) {
    fsm.trigger(TS_ACTIVATION_EVENT);
  }
}

void onEnterTractiveSystemActive() {
  if (LOG_STATE) {
    Serial.println("TS Active");
  }
}

void duringTractiveSystemActive() {
  // 1. Read RFE. If goes low go to TS-Off
  if (!RFE) {
    fsm.trigger(TS_DEACTIVATION_EVENT);
  }
  // 2. Read button. If pressed go to Manual Driving
  bool pressed = buttonPressed();
  if (pressed) {
    fsm.trigger(R2D_BUTTON_EVENT);
  }
  // 3. Send Inverter Disable
  enableInverter(false);
}

void onEnterManualDriving() {
  if (LOG_STATE) {
    Serial.println("Manual Driving");
  }
  // 1. Send Inverter Enable signal
  enableInverter(true);
}

void duringManualDriving() {
  // 1. Read RFE. If goes low go to TS-Off
  if (!RFE) {
    fsm.trigger(TS_DEACTIVATION_EVENT);
  }
  // 2. Read button. If pressed go to TS-Active
  bool pressed = buttonPressed();
  if (pressed) {
    fsm.trigger(R2D_BUTTON_EVENT);
  }
  // 3. Check APPS flags. If implausible or not receiving -> deactivate TS
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
  for(int i = 0; i < size; i++) {
    pinMode(pins[i], INPUT);
  }
}

void setupOutputs(int *pins, int size) {
  for(int i = 0; i < size; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void setup() {
  // Initialise Serial Interface
  Serial.begin(9600);

  // Initilise Pins
  // setupInputs(ANG_INP, 2); // Analog Inputs
  setupInputs(DIG_INP, 8); // Digital Inputs
  setupOutputs(DIG_OUT_5V, 8); // 5V Digital Outputs
  setupOutputs(DIG_OUT_24V, 2); // 24V Digital Outputs

  // Initialise CAN
  pinMode(CAN_BUS_STBY, OUTPUT); digitalWrite(CAN_BUS_STBY, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(handleReceive);
  Can0.mailboxStatus();

  // Setup State Machine
  setupFSM();
  onEnterTractiveSystemOff();
}

void loop() {
  fsm.run_machine();
}
