#include <SPI.h>
#include <math.h>
#include <Arduino_CAN.h>
#include <EEPROM.h>

// TODO: remove
unsigned long debugLastPrintTime = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 1000; // 1 second

// -------------------------
// Wheel speeds
// -------------------------

// TODO: move
const int WS_FR_PIN = D5;
const int WS_FL_PIN = D6;
const int PULSES_PER_REV = 12;
const int MAX_WHEEL_RPM = 2500;
const int STOP_TIMEOUT_US = 200000;
const unsigned long MIN_PULSE_SPACING_US =
  60000000UL / (PULSES_PER_REV * MAX_WHEEL_RPM);

struct WheelState {
  volatile unsigned long lastEdgeUs = 0;
  volatile unsigned long lastIntervalUs = 0;
  volatile unsigned long lastValidPulseUs = 0;
  volatile unsigned int rpm_x10 = 0;
};

WheelState flState;
WheelState frState;

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

void flISR() { capturePulse(flState); }
void frISR() { capturePulse(frState); }

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

// ***CALIBRATION VARIABLES***
bool debugSerial = false;
const uint16_t EEPROM_THROTTLE1_MIN[] = {0, 1};
const uint16_t EEPROM_THROTTLE1_MAX[] = {2, 3};
const uint16_t EEPROM_THROTTLE2_MIN[] = {4, 5};
const uint16_t EEPROM_THROTTLE2_MAX[] = {6, 7};

// APPS Variables
bool rawEnable = false;
const unsigned int THROTTLE_MARGIN = 80; // ADC Values
unsigned long throttleLastSampleTime = 0; // us
const int THROTTLE_SAMPLE_INTERVAL = 50; // 50us = 20kHz
const int THROTTLE_SAMPLE_SIZE = 10;
unsigned int throttle1Samples[THROTTLE_SAMPLE_SIZE] = {0};
unsigned int throttle2Samples[THROTTLE_SAMPLE_SIZE] = {0};
unsigned int throttleSampleIndex = 0;
unsigned long throttle1Sum = 0;
unsigned long throttle2Sum = 0;
unsigned int throttle1Val = 0;
unsigned int throttle2Val = 0;
unsigned int throttle1Raw = 0;
unsigned int throttle2Raw = 0;
unsigned int throttle1Min = 0;
unsigned int throttle1Max = 0;
unsigned int throttle2Min = 0;
unsigned int throttle2Max = 0;
bool throttleErrorFlag = false;
float throttlePercent = 0.0;
unsigned long throttleMismatchStartTime = 0;
const unsigned long THROTTLE_MISMATCH_TIMEOUT = 20; // ms
bool throttleMismatchActive = false;

// BPS Variables
const unsigned int BRAKE_MARGIN = 20;  // ADC Values
unsigned long brakeLastSampleTime = 0; // millis
const int BRAKE_SAMPLE_INTERVAL = 5; // 5ms = 200Hz
const int BRAKE_SAMPLE_SIZE = 10;
int brakeSamples[BRAKE_SAMPLE_SIZE] = {0};
unsigned int brakeSampleIndex = 0;
unsigned long brakeSum = 0;
unsigned int brakeVal = 0;
float brakePressure = 0.0; // Bar
bool brakeErrorFlag = false;

// SPI Chip Selects
const int NUM_OF_CHIP_SELECTS = 2;
const int CHIP_SELECT_0 = D1;
const int CHIP_SELECT_1 = D0;
const int ALL_CHIP_SELECTS[NUM_OF_CHIP_SELECTS] = {
  CHIP_SELECT_0,
  CHIP_SELECT_1
};

// CAN Bus Variables
static int const CAN_ID_STATUS = 0x041; // For status messages
static int const CAN_ID_SENSORS = 0x044; // Sensor Data
static int const CAN_ID_CONFIG = 0x043; // Config Message for setup/debug
static int const CAN_ID_APPS1_SETUP = 0x047; // Config message to set APPS1
static int const CAN_ID_APPS1_RAW_VALUES = 0x045; // Debug message
static int const CAN_ID_APPS2_SETUP = 0x048; // Config message to set APPS2
static int const CAN_ID_APPS2_RAW_VALUES = 0x046; // Debug message
static int const CAN_ID_TORQUE = 0x201; // For inverter to read toqrue
static int const CAN_ID_VCU_STATUS = 0x3EC; // VCU Status
static int const CAN_FILTER_MASK_STANDARD = 0x1FFC0000;

const unsigned long CAN_MESSAGE_GAP = 1; // ms minimum gap
const unsigned long TORQUE_PERIOD = 10;
const unsigned long STATUS_PERIOD = 100;
const unsigned long SENSOR_PERIOD = 100;
const unsigned long RAW_THROTTLE_PERIOD = 200;
unsigned long lastCANMessageTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastThrottle1Time = 0;
unsigned long lastThrottle2Time = 0;
unsigned long lastTorqueTime = 0;

// EV State Machine
enum EVState {
  TS_OFF = 1,
  TS_ACTIVE = 2,
  MANUAL_DRIVING = 3,
  CORSA_MODE = 7
};
EVState currentState = TS_OFF;
unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatTimeout = 200; // ms, you can increase to 250ms
unsigned long lastTorqueSendTime = 0;
const unsigned long TORQUE_SEND_INTERVAL_MS = 10; // 100 Hz
int torqueCommand = 0;

////////////////////////////////////////////////////////////////////////////

/**
 * @brief Read the value using SPI
 * 
 * @param chipSelect The chip select
 * @return unsigned int Value read
 */
unsigned int readChip(bool channel1, unsigned int chipSelect) {
  // https://github.com/souviksaha97/MCP3202/blob/master/src/MCP3202.cpp#L48
  unsigned int dataIn = 0;
  unsigned int result = 0;
  digitalWrite(chipSelect, LOW);
  uint8_t dataOut = 0b00000001;
  dataIn = SPI.transfer(dataOut);
  int dataOutChannel0 = 0b10100000;
  int dataOutChannel1 = 0b11100000;
  dataOut = channel1 ? dataOutChannel1 : dataOutChannel0;
  dataIn = SPI.transfer(dataOut);
  result = dataIn & 0x0F;
  dataIn = SPI.transfer(0x00);
  result = result << 8;
  result = result | dataIn;
  digitalWrite(chipSelect, HIGH);
  return result;
}

void handleReceive() {
  if (CAN.available()) {
    Serial.println("Received");
    CanMsg const msg = CAN.read();
    switch (msg.id) {
      // -------------------------
      // VCU heartbeat / state
      // -------------------------
      case CAN_ID_VCU_STATUS:
        currentState = static_cast<EVState>(msg.data[0]);
        lastHeartbeatTime = millis();
        break;
      // -------------------------
      // CONFIG
      // -------------------------
      case CAN_ID_CONFIG:
        rawEnable = msg.data[0] & 0x01;
        debugSerial = msg.data[0] & 0x02;
        break;
      // -------------------------
      // APPS1 setup
      // -------------------------
      case CAN_ID_APPS1_SETUP: {
        if (!rawEnable) return;
        uint16_t throttle1MinUpdate =
          (uint16_t)((msg.data[1] << 8) | msg.data[0]);
        uint16_t throttle1MaxUpdate =
          (uint16_t)((msg.data[3] << 8) | msg.data[2]);
        setThrottle1(throttle1MinUpdate, throttle1MaxUpdate);
        readThrottleSetpoints();
        break;
      }
      // -------------------------
      // APPS2 setup
      // -------------------------
      case CAN_ID_APPS2_SETUP: {
        if (!rawEnable) return;
        uint16_t throttle2MinUpdate =
          (uint16_t)((msg.data[1] << 8) | msg.data[0]);
        uint16_t throttle2MaxUpdate =
          (uint16_t)((msg.data[3] << 8) | msg.data[2]);
        setThrottle2(throttle2MinUpdate, throttle2MaxUpdate);
        readThrottleSetpoints();
        break;
      }
      default:
        break;
    }
  }

  // -------------------------
  // Fail-safe heartbeat timeout
  // -------------------------
  if (millis() - lastHeartbeatTime > heartbeatTimeout) {
    currentState = TS_OFF;

    if (debugSerial) {
      Serial.println("[CAN] Heartbeat timeout -> TS_OFF");
    }
  }
}

int16_t readEEPROM16(const uint16_t address[]) {
  uint8_t lowByte = EEPROM.read(address[0]);
  uint8_t highByte = EEPROM.read(address[1]);
  return (int16_t)((highByte << 8) | lowByte);
}

void writeEEPROM16(const uint16_t address[], int16_t value) {
  uint8_t lowByte = value & 0xFF;
  uint8_t highByte = (value >> 8) & 0xFF;
  EEPROM.write(address[0], lowByte);
  EEPROM.write(address[1], highByte);
}

void readThrottleSetpoints() {
  throttle1Min = readEEPROM16(EEPROM_THROTTLE1_MIN);
  throttle1Max = readEEPROM16(EEPROM_THROTTLE1_MAX);
  throttle2Min = readEEPROM16(EEPROM_THROTTLE2_MIN);
  throttle2Max = readEEPROM16(EEPROM_THROTTLE2_MAX);
}

void setThrottle1(int16_t minValue, int16_t maxValue) {
  int16_t storedMin;
  int16_t storedMax;
  // Read existing values
  storedMin = readEEPROM16(EEPROM_THROTTLE1_MIN);
  storedMax = readEEPROM16(EEPROM_THROTTLE1_MAX);
  // Update only if values changed
  if (storedMin != minValue) {
    writeEEPROM16(EEPROM_THROTTLE1_MIN, minValue);
  }
  if (storedMax != maxValue) {
    writeEEPROM16(EEPROM_THROTTLE1_MAX, maxValue);
  }
}

void setThrottle2(int16_t minValue, int16_t maxValue) {
  int16_t storedMin;
  int16_t storedMax;
  // Read existing values
  storedMin = readEEPROM16(EEPROM_THROTTLE2_MIN);
  storedMax = readEEPROM16(EEPROM_THROTTLE2_MAX);
  // Update only if values changed
  if (storedMin != minValue) {
    writeEEPROM16(EEPROM_THROTTLE2_MIN, minValue);
  }
  if (storedMax != maxValue) {
    writeEEPROM16(EEPROM_THROTTLE2_MAX, maxValue);
  }
}

//----------------
// Throttle Pedal
//----------------

void readThrottle() {
  if (micros() - throttleLastSampleTime >= THROTTLE_SAMPLE_INTERVAL) {
    throttleLastSampleTime = micros();
    // Read raw sensor values
    throttle1Raw = readChip(false, CHIP_SELECT_1);
    throttle2Raw = readChip(true, CHIP_SELECT_1);
    // Remove oldest samples from the sum
    throttle1Sum -= throttle1Samples[throttleSampleIndex];
    throttle2Sum -= throttle2Samples[throttleSampleIndex];
    // Add new samples
    throttle1Samples[throttleSampleIndex] = throttle1Raw;
    throttle2Samples[throttleSampleIndex] = throttle2Raw;
    throttle1Sum += throttle1Raw;
    throttle2Sum += throttle2Raw;
    // Advance buffer index
    throttleSampleIndex = (throttleSampleIndex + 1) % THROTTLE_SAMPLE_SIZE;
    // Compute filtered value
    throttle1Val = (unsigned int) throttle1Sum / THROTTLE_SAMPLE_SIZE;
    throttle2Val = (unsigned int) throttle2Sum / THROTTLE_SAMPLE_SIZE;
  }
}

void checkThrottleSCS() {
  throttleErrorFlag = false;
  // Sensor 1 range check
  if (abs((int)throttle1Val - (int)throttle1Min) <= THROTTLE_MARGIN) throttle1Val = throttle1Min;
  if (abs((int)throttle1Val - (int)throttle1Max) <= THROTTLE_MARGIN) throttle1Val = throttle1Max;
  // Sensor 2 range check
  if (abs((int)throttle2Val - (int)throttle2Min) <= THROTTLE_MARGIN) throttle2Val = throttle2Min;
  if (abs((int)throttle2Val - (int)throttle2Max) <= THROTTLE_MARGIN) throttle2Val = throttle2Max;
  // Out of range fault
  if (throttle1Val < throttle1Min || throttle1Val > throttle1Max) {
    throttleErrorFlag = true;
    throttle1Val = throttle1Min;
  }
  if (throttle2Val < throttle2Min || throttle2Val > throttle2Max) {
    throttleErrorFlag = true;
    throttle2Val = throttle2Max;
  }
}

void calculateThrottle() {
  float throttle1Percent = ((float)(throttle1Val - throttle1Min) 
    / (float)(throttle1Max - throttle1Min)) * 100.0;
  float throttle2Percent = ((float)(throttle2Max - throttle2Val) 
    / (float)(throttle2Max - throttle2Min)) * 100.0;
  // Check throttle similarity
  if (abs(throttle1Percent - throttle2Percent) >= 10.0) {
    if (throttleMismatchStartTime == 0) {
      throttleMismatchStartTime = millis();
    }
    if (millis() - throttleMismatchStartTime >= THROTTLE_MISMATCH_TIMEOUT) {
      throttleErrorFlag = true;
      throttleMismatchActive = true;
    }
  } else {
    // Sensors agree again
    throttleMismatchStartTime = 0;
    throttleMismatchActive = false;
  }
  // Average redundant sensors
  throttlePercent = (throttle1Percent + throttle2Percent) / 2.0;
  // Clamp final result
  throttlePercent = constrain(throttlePercent, 0, 100);
}

//----------------
// Brake Pressure
//----------------
const int BRAKE_MIN_SCS = (0.4/5.0) * 4095;
const int BRAKE_MIN_VAL = (0.5/5.0) * 4095;
const int BRAKE_MAX_VAL = (4.5/5.0) * 4095;
const int BRAKE_MAX_SCS = (4.6/5.0) * 4095;

void readBrakes() {
  if (millis() - brakeLastSampleTime >= BRAKE_SAMPLE_INTERVAL) {
    brakeLastSampleTime = millis();
    // Read raw sensor value
    unsigned int brakeRaw = readChip(false, CHIP_SELECT_0);
    // Remove oldest sample from sum
    brakeSum -= brakeSamples[brakeSampleIndex];
    // Add new sample
    brakeSamples[brakeSampleIndex] = brakeRaw;
    brakeSum += brakeRaw;
    // Advance buffer index
    brakeSampleIndex = (brakeSampleIndex + 1) % BRAKE_SAMPLE_SIZE;
    // Compute filtered value
    brakeVal = brakeSum / BRAKE_SAMPLE_SIZE;
  }
}

void checkBrakesSCS() {
  brakeErrorFlag = false;
  if (brakeVal >= BRAKE_MAX_SCS) brakeErrorFlag = true;
  if (brakeVal >= BRAKE_MAX_VAL) brakeVal = BRAKE_MAX_VAL;
  if (brakeVal <= BRAKE_MIN_SCS) brakeErrorFlag = true;
  if (brakeVal <= BRAKE_MIN_VAL) brakeVal = BRAKE_MIN_VAL;
}

void calculateBrakePressure() {
  // Calculate Brake Pressure
  brakePressure = ((brakeVal - BRAKE_MIN_VAL) * (5.0/4095) * 1000.0) / 15.38;
  // Clamp final result
  brakePressure = constrain(brakePressure, 0, 260);
}

//----------------
// Inverter Torque Commands
//----------------
const float BRAKE_PRESSURE_THRESHOLD = 3.0; // bar
unsigned long brakeThrottleStartTime = 0; // millis
const int THROTTLE_BRAKE_TIME = 500; // ms
const float THROTTLE_BRAKE_LIMIT = 25.0; // %
bool brakeThrottleFault = false;
bool pedalActive = false;

void checkBrakeThrottle() {
  // If fault already active, wait for reset condition
  if (brakeThrottleFault) {
    if (throttlePercent < 5.0) {
      brakeThrottleFault = false;
      brakeThrottleStartTime = 0;
    }
    return;
  }

  // Detect brake + throttle conflict
  if ((brakePressure > BRAKE_PRESSURE_THRESHOLD) &&
      (throttlePercent > THROTTLE_BRAKE_LIMIT)) {
    if (brakeThrottleStartTime == 0) brakeThrottleStartTime = millis();
    if (millis() - brakeThrottleStartTime >= THROTTLE_BRAKE_TIME) {
      brakeThrottleFault = true;
    }
  } else {
    // Condition has cleared before timeout
    brakeThrottleStartTime = 0;
  }
}

void calculateTorque() {
  if (throttleErrorFlag || brakeErrorFlag || brakeThrottleFault) {
    pedalActive = false;
    torqueCommand = 0;
    return;
  }
  torqueCommand = (throttlePercent / 100.0) * -30000;
}

//----------------
// CAN Messages
//----------------

void processCAN() {
  unsigned long now = millis();
  // Enforce gap between any two CAN messages
  if (now - lastCANMessageTime < CAN_MESSAGE_GAP) {
    return;
  }

  if ((currentState == MANUAL_DRIVING) && (now - lastTorqueTime >= TORQUE_PERIOD)) {
    sendTorqueRequest();
    lastTorqueTime = now;
    lastCANMessageTime = now;
    return;
  }

  if (now - lastStatusTime >= STATUS_PERIOD) {
    sendStatus();
    lastStatusTime = now;
    lastCANMessageTime = now;
    return;
  }

  if (now - lastSensorTime >= SENSOR_PERIOD) {
    sendSensors();
    lastSensorTime = now;
    lastCANMessageTime = now;
    return;
  }

  if (rawEnable && (now - lastThrottle1Time >= RAW_THROTTLE_PERIOD)) {
    sendThrottle1Raw();
    lastThrottle1Time = now;
    lastCANMessageTime = now;
    return;
  }
  if (rawEnable && (now - lastThrottle2Time >= RAW_THROTTLE_PERIOD)) {
    sendThrottle2Raw();
    lastThrottle2Time = now;
    lastCANMessageTime = now;
    return;
  }
}

void sendTorqueRequest() {
  uint8_t torqueLow = torqueCommand & 0xFF;
  uint8_t torqueHigh = (torqueCommand >> 8) & 0xFF;
  // 0x90 = torque mode multiplex for inverter
  uint8_t msg_data_torque[] = {0x90, torqueLow, torqueHigh};
  CanMsg const msg(
    CanStandardId(CAN_ID_TORQUE),
    sizeof(msg_data_torque),
    msg_data_torque
  );
  CAN.write(msg);
}

void sendStatus() {
  uint8_t appsOutput = throttlePercent;
  uint8_t statusFlags = 0;
  statusFlags |= (brakeErrorFlag & 0x01) << 0;
  statusFlags |= (0 & 0x01) << 1;
  statusFlags |= (throttleErrorFlag & 0x01) << 2;
  statusFlags |= (pedalActive & 0x01) << 3;
  statusFlags |= (brakeThrottleFault & 0x01) << 4;

  uint8_t msg_data_status[] = {appsOutput, statusFlags};
  CanMsg const msg(CanStandardId(CAN_ID_STATUS), sizeof(msg_data_status), msg_data_status);
  CAN.write(msg);
}

void sendSensors() {
  int16_t brakePressureCommand = (int16_t)(brakePressure);
  uint8_t bpsLow = brakePressureCommand & 0xFF;
  uint8_t bpsHigh = (brakePressureCommand >> 8) & 0xFF;


  unsigned int flRPM_x10;
  unsigned int frRPM_x10;
  // Copy safely from interrupt variables
  noInterrupts();
  flRPM_x10 = flState.rpm_x10;
  frRPM_x10 = frState.rpm_x10;
  interrupts();
  // Split into CAN bytes
  uint8_t flLow = flRPM_x10 & 0xFF;
  uint8_t flHigh = (flRPM_x10 >> 8) & 0xFF;
  uint8_t frLow = frRPM_x10 & 0xFF;
  uint8_t frHigh = (frRPM_x10 >> 8) & 0xFF;

  uint8_t msg_data_sensors[] = {bpsLow, bpsHigh, 0, 0, flLow, flHigh, frLow, frHigh};
  CanMsg const msg(CanStandardId(CAN_ID_SENSORS), sizeof(msg_data_sensors), msg_data_sensors);
  CAN.write(msg);
}

void sendThrottle1Raw() {
  uint8_t throttleLow = throttle1Raw & 0xFF;
  uint8_t throttleHigh = (throttle1Raw >> 8) & 0xFF;
  uint8_t throttleMinLow = throttle1Min & 0xFF;
  uint8_t throttleMinHigh = (throttle1Min >> 8) & 0xFF;
  uint8_t throttleMaxLow = throttle1Max & 0xFF;
  uint8_t throttleMaxHigh = (throttle1Max >> 8) & 0xFF;

  uint8_t msg_data_throttle_raw[] = {throttleLow, throttleHigh, throttleMinLow, throttleMinHigh, throttleMaxLow, throttleMaxHigh};
  CanMsg const msg(CanStandardId(CAN_ID_APPS1_RAW_VALUES), sizeof(msg_data_throttle_raw), msg_data_throttle_raw);
  CAN.write(msg);
}

void sendThrottle2Raw() {
  uint8_t throttleLow = throttle2Raw & 0xFF;
  uint8_t throttleHigh = (throttle2Raw >> 8) & 0xFF;
  uint8_t throttleMinLow = throttle2Min & 0xFF;
  uint8_t throttleMinHigh = (throttle2Min >> 8) & 0xFF;
  uint8_t throttleMaxLow = throttle2Max & 0xFF;
  uint8_t throttleMaxHigh = (throttle2Max >> 8) & 0xFF;

  uint8_t msg_data_throttle_raw[] = {throttleLow, throttleHigh, throttleMinLow, throttleMinHigh, throttleMaxLow, throttleMaxHigh};
  CanMsg const msg(CanStandardId(CAN_ID_APPS2_RAW_VALUES), sizeof(msg_data_throttle_raw), msg_data_throttle_raw);
  CAN.write(msg);
}

//----------------
// Logging
//----------------

void logConsole() {
  if (millis() - debugLastPrintTime >= DEBUG_PRINT_INTERVAL) {
    debugLastPrintTime = millis();
    Serial.println("----- System Status -----");
    // Error flags
    Serial.print("Throttle Error: ");
    Serial.println(throttleErrorFlag);
    Serial.print("Brake Error: ");
    Serial.println(brakeErrorFlag);
    Serial.print("Brake/Throttle Fault: ");
    Serial.println(brakeThrottleFault);
    // Throttle sensors
    Serial.print("Throttle 1 ADC: ");
    Serial.println(throttle1Val);
    Serial.print("Throttle 2 ADC: ");
    Serial.println(throttle2Val);
    Serial.print("Throttle %: ");
    Serial.println(throttlePercent);
    // Brake
    Serial.print("Brake Pressure: ");
    Serial.println(brakePressure);
    // Torque
    Serial.print("Torque Request: ");
    Serial.println(torqueCommand);
    Serial.println("-------------------------");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  
  // Setup chip selects
  for (int i = 0; i < NUM_OF_CHIP_SELECTS; i++) {
    pinMode(ALL_CHIP_SELECTS[i], OUTPUT);
    // LOW to select chip
    digitalWrite(ALL_CHIP_SELECTS[i], HIGH);
  }
  // Setup SPI
  SPI.begin();

  // CAN Bus Standby
  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);
  // Setup CAN Bus
  CAN.setFilterMask_Standard(CAN_FILTER_MASK_STANDARD);
  CAN.setFilterId_Standard(0, CAN_ID_VCU_STATUS);
  CAN.setFilterId_Standard(1, CAN_ID_CONFIG);
  CAN.setFilterId_Standard(2, CAN_ID_APPS1_SETUP);
  CAN.setFilterId_Standard(3, CAN_ID_APPS2_SETUP);
  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  // Read Throttle setpoints
  readThrottleSetpoints();

  // Setup Wheelspeeds
  pinMode(WS_FL_PIN, INPUT);
  pinMode(WS_FR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WS_FL_PIN), flISR, RISING);
  attachInterrupt(digitalPinToInterrupt(WS_FR_PIN), frISR, RISING);
}

void loop() {
  handleReceive(); // Handle CAN receive

  // Wheelspeed
  updateWheelSpeed(flState);
  updateWheelSpeed(frState);
  
  // APPS
  readThrottle();
  checkThrottleSCS();
  calculateThrottle();

  // Brake Pressure
  readBrakes();
  checkBrakesSCS();
  calculateBrakePressure();

  // Send CAN Messages
  if (currentState == MANUAL_DRIVING) {
    pedalActive = true;
    checkBrakeThrottle();
    calculateTorque();
  } else {
    pedalActive = false;
  }
  processCAN();
  if (debugSerial) logConsole();
}