#include <Fsm.h>
#include <SPI.h>


// Pins
const int CHIP_SELECT_PIN = D7;
const int AIR_PLUS = D0;
const int AIR_MINUS = D1;
const int PRE_CHARGE = D2;
const int DISCHARGE_DISABLE = D3;
const int SDC_PG = D4;

// Input Buttons:
// const int SPARE_INPUT = D5; // Will be used for autonomous TS-Activation
const int ACTIVATION_BUTTON = D6;  // TS-Activation button input pin
int activationButtonState; // current button reading
int lastActivationButtonState = LOW;  // previous button reading

// Timers
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastActivationDebounceTime = 0;  // the last time the output pin was toggled
const unsigned long DEBOUNCE_DELAY = 20;   // the debounce time; increase if the output flickers
int count = 0;
bool sdcPowerGood = false;
const unsigned long CONTACTOR_DELAY = 200; // 200

// Print Debug
const bool LOG_STATE = true;
const bool LOG_VOLT = true;
const unsigned long LOG_VOLT_TIME = 500;
unsigned long lastVoltLog = 0; // Last time voltage was logged

//Events
#define BUTTON_EVENT  0
#define SDC_EVENT  1
#define PRE_CHARGE_COMPLETE  2

// FSM States
State stateIdle(&onEnterStateIdle, NULL, NULL);
State stateSDC(&onEnterSDC, NULL, NULL);
Fsm fsm(&stateIdle);

void onEnterStateIdle() {
  Serial.println("IDLE");
  digitalWrite(DISCHARGE_DISABLE, LOW);
  digitalWrite(AIR_MINUS, LOW);
  digitalWrite(PRE_CHARGE, LOW);
  digitalWrite(AIR_PLUS, LOW);
}

void onEnterSDC() {
  if (LOG_STATE) {
    Serial.println("SDC Opened");
  }
}

void setupFSM() {
  fsm.add_transition(&stateTractiveSystemActive, &stateSDC, SDC_EVENT, NULL);
  fsm.add_timed_transition(&stateSDC, &stateIdle, 200, NULL);
  fsm.add_transition(&stateCheckVoltage, &stateAirPlusClosed, PRE_CHARGE_COMPLETE, NULL);
}

void setup() {
  Serial.begin(9600);

  // Setup Contactor Outputs
  pinMode(AIR_PLUS, OUTPUT);
  pinMode(AIR_MINUS, OUTPUT);
  pinMode(PRE_CHARGE, OUTPUT);
  pinMode(DISCHARGE_DISABLE, OUTPUT);
  
  // Setup Inputs
  pinMode(SDC_PG, INPUT);
  pinMode(ACTIVATION_BUTTON, INPUT);

  // Setup SPI
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  SPI.begin();

  // Setup State Machine
  setupFSM();
  onEnterStateIdle(); // Ensure outputs start LOW
}

void loop() {
  // Handle Activation Button
  int activationButtonReading = digitalRead(ACTIVATION_BUTTON);
  if (activationButtonReading != lastActivationButtonState) {
    lastActivationDebounceTime = millis(); // Reset debounce timer
  }
  if ((millis() - lastActivationDebounceTime) > DEBOUNCE_DELAY) {
    if (activationButtonReading != activationButtonState) {
      activationButtonState = activationButtonReading;
      if (activationButtonState == LOW) { // Button Released
        if (sdcPowerGood) {
          count++;
          fsm.trigger(BUTTON_EVENT);
        }
      }
    }
  }
  lastActivationButtonState = activationButtonReading;

  // check that SDC is closed
  sdcPowerGood = digitalRead(SDC_PG); // SDC activated when SDC_PG >19.5V
  if (!sdcPowerGood)  {
    fsm.trigger(SDC_EVENT);
  }
  
  fsm.run_machine();
}
