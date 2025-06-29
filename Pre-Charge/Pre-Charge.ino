#include <Fsm.h>
#include <SPI.h>

// Voltage Measurement
const int Vdd = 5; // ADC Power Supply
const float VOLTAGE_THRESHOLD = 50; // V

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

// Double Press:
// const unsigned long doublePressWindow = 400; // Max time between presses for double press
// bool buttonState = HIGH;         // Current stable button state
// bool lastButtonReading = HIGH;   // Last raw button reading
// unsigned long lastDebounceTime = 0;
// unsigned long lastPressTime = 0;
// bool waitingForSecondPress = false;

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
State stateDischargeDisabled(&onEnterStateDischargeDisabled, NULL, NULL);
State stateAirMinusClosed(&onEnterStateAirMinusClosed, NULL, NULL);
State statePreCharge(&onEnterStatePreCharge, NULL, NULL);
State stateCheckVoltage(&onEnterStateCheckVoltage, &duringStateCheckVoltage, NULL);
State stateDelayHV(&onEnterStateDelayHV, &duringStateDelayHV, NULL);
State stateAirPlusClosed(&onEnterStateAirPlusClosed, NULL, NULL);
State stateTractiveSystemActive(&onEnterStateTractiveSystemActive, NULL, NULL);
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

void onEnterStateDischargeDisabled() {
  if (LOG_STATE) {
    Serial.println("Disabling Discharge");
  }
  digitalWrite(DISCHARGE_DISABLE, HIGH);
}

void onEnterStateAirMinusClosed() {
  if (LOG_STATE) {
    Serial.println("Closing AIR_MINUS");
  }
  digitalWrite(AIR_MINUS, HIGH);
}

void onEnterStatePreCharge() {
  if (LOG_STATE) {
    Serial.println("Closing Pre-Charge Contactor");
  }
  digitalWrite(PRE_CHARGE, HIGH);
}

void onEnterStateCheckVoltage() {
  if (LOG_STATE) {
    Serial.println("Starting Voltage Measurement");
  }
}

unsigned int readADC(int channel) {
  // https://github.com/souviksaha97/MCP3202/blob/master/src/MCP3202.cpp#L48
  unsigned int dataIn = 0;
  unsigned int result = 0;
  digitalWrite(CHIP_SELECT_PIN, LOW);
  uint8_t dataOut = 0b00000001;
  dataIn = SPI.transfer(dataOut);
  int dataOutChannel0 = 0b10100000;
  int dataOutChannel1 = 0b11100000;
  dataOut = (channel == 0) ? dataOutChannel1 : dataOutChannel0;
  dataIn = SPI.transfer(dataOut);
  result = dataIn & 0x0F;
  dataIn = SPI.transfer(0x00);
  result = result << 8;
  result = result | dataIn;
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  return result;        
}

float calculateVoltage(unsigned int reading) {
  if ((reading <= 50) || (reading >= 4050)) {
    reading = 0; // ADC not connected - Reading noise
  }

  float rawVoltage = reading * (Vdd / 4096.0);
  float scalingFactor = ((470.0*5 + 37.0)/37.0); // Resistor Divider
  float voltage = rawVoltage * scalingFactor;
  
  return voltage;
}

void duringStateCheckVoltage() {
  unsigned int accumulatorReading = readADC(0);
  unsigned int inverterReading = readADC(1);
  float accumulatorVoltage = calculateVoltage(accumulatorReading);
  float inverterVoltage = calculateVoltage(inverterReading);

  if (LOG_VOLT && (millis() - lastVoltLog) > LOG_VOLT_TIME) {
    lastVoltLog = millis();
    Serial.print("Accumulator:");
    Serial.print(accumulatorVoltage);
    Serial.println("V");
    Serial.print("Inverter:");
    Serial.print(inverterVoltage);
    Serial.println("V");
  }

  if ((accumulatorVoltage > VOLTAGE_THRESHOLD) && (inverterReading <= accumulatorReading) && (inverterVoltage >= 0.95*accumulatorVoltage)) {
    Serial.print("Accumulator:");
    Serial.print(accumulatorVoltage);
    Serial.println("V");
    Serial.print("Inverter:");
    Serial.print(inverterVoltage);
    Serial.println("V");
    Serial.println("Pre-Charge Voltage Reached");
    fsm.trigger(PRE_CHARGE_COMPLETE);
  }
}

void onEnterStateDelayHV() {
  if (LOG_STATE) {
    Serial.println("Pre-Charge Completed");
  }
}

void duringStateDelayHV() {
    unsigned int accumulatorReading = readADC(0);
    unsigned int inverterReading = readADC(1);
    float accumulatorVoltage = calculateVoltage(accumulatorReading);
    float inverterVoltage = calculateVoltage(inverterReading);

    if (LOG_VOLT && (millis() - lastVoltLog) > LOG_VOLT_TIME) {
      lastVoltLog = millis();
      Serial.print("Accumulator:");
      Serial.print(accumulatorVoltage);
      Serial.println("V");
      Serial.print("Inverter:");
      Serial.print(inverterVoltage);
      Serial.println("V");
    }
}

void onEnterStateAirPlusClosed() {
  if (LOG_STATE) {
    Serial.println("Closing AIR_PLUS");
  }
  digitalWrite(AIR_PLUS, HIGH);
}

void onEnterStateTractiveSystemActive() {
  if (LOG_STATE) {
    Serial.println("Tractive System Active; Opening Pre-Charge");
  }
  digitalWrite(PRE_CHARGE, LOW);
}

void setupFSM() {
  fsm.add_transition(&stateIdle, &stateDischargeDisabled, BUTTON_EVENT, NULL);
  fsm.add_transition(&stateDischargeDisabled, &stateIdle, BUTTON_EVENT, NULL);
  fsm.add_transition(&stateAirMinusClosed, &stateIdle, BUTTON_EVENT, NULL);
  fsm.add_transition(&statePreCharge, &stateIdle, BUTTON_EVENT, NULL);
  fsm.add_transition(&stateCheckVoltage, &stateIdle, BUTTON_EVENT, NULL);
  fsm.add_transition(&stateAirPlusClosed, &stateIdle, BUTTON_EVENT, NULL);
  fsm.add_transition(&stateTractiveSystemActive, &stateIdle, BUTTON_EVENT, NULL);
  fsm.add_timed_transition(&stateDischargeDisabled, &stateAirMinusClosed, CONTACTOR_DELAY, NULL);
  fsm.add_timed_transition(&stateAirMinusClosed, &statePreCharge, CONTACTOR_DELAY, NULL);
  fsm.add_timed_transition(&statePreCharge, &stateCheckVoltage, CONTACTOR_DELAY, NULL);
  fsm.add_timed_transition(&stateAirPlusClosed, &stateTractiveSystemActive, 500, NULL);

  fsm.add_transition(&stateDischargeDisabled, &stateSDC, SDC_EVENT, NULL);
  fsm.add_transition(&stateAirMinusClosed, &stateSDC, SDC_EVENT, NULL);
  fsm.add_transition(&statePreCharge, &stateSDC, SDC_EVENT, NULL);
  fsm.add_transition(&stateCheckVoltage, &stateSDC, SDC_EVENT, NULL);
  fsm.add_transition(&stateAirPlusClosed, &stateSDC, SDC_EVENT, NULL);
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

  // Alt - Double Press:
  // bool currentReading = digitalRead(ACTIVATION_BUTTON);
  // unsigned long currentTime = millis();

  // // Debounce logic
  // if (currentReading != lastButtonReading) {
  //   lastDebounceTime = currentTime;
  //   lastButtonReading = currentReading;
  // }

  // if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
  //   // State has been stable for debounceDelay
  //   if (currentReading != buttonState) {
  //     buttonState = currentReading;

  //     // Button was just pressed (transition from HIGH to LOW)
  //     if (buttonState == LOW) {
  //       if (waitingForSecondPress && (currentTime - lastPressTime <= doublePressWindow)) {
  //         if (sdcPowerGood) {
  //           count++;
  //           fsm.trigger(BUTTON_EVENT);
  //         }
  //         waitingForSecondPress = false;
  //       } else {
  //         waitingForSecondPress = true;
  //         lastPressTime = currentTime;
  //       }
  //     }
  //   }
  // }
  // // Timeout waiting for second press
  // if (waitingForSecondPress && (currentTime - lastPressTime > doublePressWindow)) {
  //   waitingForSecondPress = false;
  // }


  // check that SDC is closed
  sdcPowerGood = digitalRead(SDC_PG); // SDC activated when SDC_PG >19.5V
  if (!sdcPowerGood)  {
    fsm.trigger(SDC_EVENT);
  }
  
  fsm.run_machine();
}
