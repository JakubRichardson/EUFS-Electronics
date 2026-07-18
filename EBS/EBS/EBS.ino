#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Fsm.h>
#include <SPI.h>

// Note: Air Pressure sensors not working correctly, so code was modified to accomodate this

// Logging
bool LOG_STATE = true;
bool LOG_TIMEOUT = true;

// EBS State Request
enum EBS_Request {
  REQ_OFF = 0,
  REQ_ON = 1,
  REQ_TEST = 2
};
EBS_Request requestedState = REQ_OFF;

// EBS Status
enum EBS_Status {
  EBS_OFF = 0,
  EBS_ON = 1,
  EBS_ERROR = 2
};
EBS_Status ebsStatus = EBS_OFF;
bool ebsTestComplete = false;

// SDC
bool sdcStatus = false;

// Pin definitions
const int HEARTBEAT_PIN = 2;
const int WD_reset  = 3;
const int WD_status = 4;
const int AS_close_SDC = 5;
const int CAN_STBY = 6;
const int EBS_enbl_rear = 7;
const int EBS_enbl_front = 8;
const int SDC_IN_status = 19;
const int SDC_OUT_status = 20;

// BPS Variables
const float BRAKE_THRESHOLD_DISENGAGED = 1.0; // Bar
const float BRAKE_THRESHOLD_ENGAGED = 20.0; // Bar
unsigned long brakeLastSampleTime = 0; // millis
const int BRAKE_SAMPLE_INTERVAL = 5; // 5ms = 200Hz
const int BRAKE_SAMPLE_SIZE = 10;
int brakeSamplesFront[BRAKE_SAMPLE_SIZE] = {0};
int brakeSamplesRear[BRAKE_SAMPLE_SIZE] = {0};
unsigned int brakeSampleIndex = 0;
unsigned long brakeSumFront = 0;
unsigned long brakeSumRear = 0;
unsigned int brakeValFront = 0;
unsigned int brakeValRear = 0;
unsigned int brakeRawFront = 0;
unsigned int brakeRawRear = 0;
float brakePressureFront = 0.0; // Bar
float brakePressureRear = 0.0; // Bar
bool brakeErrorFlagFront = false;
bool brakeErrorFlagRear = false;

// APS Variables
const int AIR_THRESHOLD_MIN = 2; // Bar
unsigned long airLastSampleTime = 0; // millis
const int AIR_SAMPLE_INTERVAL = 5; // 5ms = 200Hz
const int AIR_SAMPLE_SIZE = 10;
int airSamplesFront[AIR_SAMPLE_SIZE] = {0};
int airSamplesRear[AIR_SAMPLE_SIZE] = {0};
unsigned int airSampleIndex = 0;
unsigned long airSumFront = 0;
unsigned long airSumRear = 0;
unsigned int airValFront = 0;
unsigned int airValRear = 0;
float airPressureFront = 0.0; // Bar
float airPressureRear = 0.0; // Bar
bool airErrorFlagFront = false;
bool airErrorFlagRear = false;

// CAN Bus Variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;
static int const CAN_ID_VCU_STATUS = 0x3EC; // VCU Status
static int const CAN_ID_STATUS = 0x050; // Status message
static int const CAN_ID_SENSORS = 0x051; // Sensor Data
const unsigned long CAN_MESSAGE_GAP = 1; // ms minimum gap
const unsigned long STATUS_PERIOD = 100;
const unsigned long SENSORS_PERIOD = 100;
unsigned long lastCANMessageTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSensorsTime = 0;

// SPI Chip Selects
const int NUM_OF_CHIP_SELECTS = 2;
const int CHIP_SELECT_0 = 9;
const int CHIP_SELECT_1 = 10;
const int ALL_CHIP_SELECTS[NUM_OF_CHIP_SELECTS] = {
  CHIP_SELECT_0,
  CHIP_SELECT_1
};

// Watchdog Test
bool watchdogTested = false;
unsigned int MAX_WATCHDOG_DELAY = 250; // ms

// Heartbeat timing
bool runHeartbeat = false;
unsigned long lastHeartbeatToggle = 0;
const unsigned long heartbeatInterval = 20; // ms (Period/2)
bool heartbeatState = false;

// Watchdog Reset Timing
unsigned long resetTimer = 0;
bool delayComplete = false;
bool pulseFinished = false;
const unsigned long resetDelay = 240; // ms before WD_reset pulse
const unsigned long pulseWidth = 2;   // ms pulse high time
const unsigned long settleTime = 8;   // ms after LOW before transition

// Events
#define ERROR 0
#define START_HEARTBEAT 1
#define VERIFY_HEARTBEAT 2
#define TEST_HEARTBEAT 3
#define CLOSE_SDC 4
#define CHECK_PRESSURE 5
#define DEACTIVATE_REAR 6
#define REACTIVATE_REAR 7
#define DEACTIVATE_FRONT 8
#define REACTIVATE_FRONT 9
#define DRIVING 10
#define RESET 11

// FSM States
State stateError(&onEnterError, &duringError, NULL);
State stateIdle(&onEnterIdle, &duringIdle, NULL);
State stateStartHeartbeat(&onEnterStartHeartbeat, &duringStartHeartbeat, NULL);
State stateCheckStatus(&onEnterCheckStatus, &duringCheckStatus, NULL);
State stateTestWatchdog(&onEnterTestWatchdog, &duringTestWatchdog, NULL);
State stateCloseSDC(&onEnterCloseSDC, &duringCloseSDC, NULL);
State stateCheckPressure(&onEnterCheckPressure, &duringCheckPressure, NULL);
State stateDeactivateRear(&onEnterDeactivateRear, &duringDeactivateRear, NULL);
State stateReactivateRear(&onEnterReactivateRear, &duringReactivateRear, NULL);
State stateDeactivateFront(&onEnterDeactivateFront, &duringDeactivateFront, NULL);
State stateReactivateFront(&onEnterReactivateFront, &duringReactivateFront, NULL);
State stateDeactivateEBS(&onEnterDeactivateEBS, NULL, NULL);
State stateMonitoring(&onEnterMonitoring, &duringMonitoring, NULL);
Fsm fsm(&stateIdle);

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

// Heartbeat update
void updateHeartbeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatToggle >= heartbeatInterval) {
    lastHeartbeatToggle = currentMillis;
    heartbeatState = !heartbeatState;
    digitalWrite(HEARTBEAT_PIN, heartbeatState);
  }
}

//----------------
// Brake Pressure
//----------------
const unsigned int BRAKE_MIN_SCS = (0.2/5.0) * 4095;
const unsigned int BRAKE_MIN_VAL = (0.5/5.0) * 4095;
const unsigned int BRAKE_MAX_VAL = (4.5/5.0) * 4095;
const unsigned int BRAKE_MAX_SCS = (4.8/5.0) * 4095;

void readBrakes() {
  brakeLastSampleTime = millis();
  // Read raw sensor value
  brakeRawFront = readChip(true, CHIP_SELECT_0);
  brakeRawRear = readChip(false, CHIP_SELECT_0);
  // Remove oldest sample from sum
  brakeSumFront -= brakeSamplesFront[brakeSampleIndex];
  brakeSumRear -= brakeSamplesRear[brakeSampleIndex];
  // Add new sample
  brakeSamplesFront[brakeSampleIndex] = brakeRawFront;
  brakeSamplesRear[brakeSampleIndex] = brakeRawRear;
  brakeSumFront += brakeRawFront;
  brakeSumRear += brakeRawRear;
  // Advance buffer index
  brakeSampleIndex = (brakeSampleIndex + 1) % BRAKE_SAMPLE_SIZE;
  // Compute filtered value
  brakeValFront = brakeSumFront / BRAKE_SAMPLE_SIZE;
  brakeValRear = brakeSumRear / BRAKE_SAMPLE_SIZE;
}

void checkBrakesSCS() {
  brakeErrorFlagFront = false;
  if (brakeValFront >= BRAKE_MAX_SCS) brakeErrorFlagFront = true;
  if (brakeValFront >= BRAKE_MAX_VAL) brakeValFront = BRAKE_MAX_VAL;
  if (brakeValFront <= BRAKE_MIN_SCS) brakeErrorFlagFront = true;
  if (brakeValFront <= BRAKE_MIN_VAL) brakeValFront = BRAKE_MIN_VAL;
  
  brakeErrorFlagRear = false;
  if (brakeValRear >= BRAKE_MAX_SCS) brakeErrorFlagRear = true;
  if (brakeValRear >= BRAKE_MAX_VAL) brakeValRear = BRAKE_MAX_VAL;
  if (brakeValRear <= BRAKE_MIN_SCS) brakeErrorFlagRear = true;
  if (brakeValRear <= BRAKE_MIN_VAL) brakeValRear = BRAKE_MIN_VAL;
}

void calculateBrakePressure() {
  // Calculate Brake Pressure
  brakePressureFront = ((brakeValFront - BRAKE_MIN_VAL) * (5.0/4095) * 1000.0) / 28.57;
  brakePressureRear = ((brakeValRear - BRAKE_MIN_VAL) * (5.0/4095) * 1000.0) / 28.57;
  // Clamp final result
  brakePressureFront = constrain(brakePressureFront, 0, 140);
  brakePressureRear = constrain(brakePressureRear, 0, 140);
}


//----------------
// Air Pressure
//----------------
const int AIR_MIN_SCS = (0.4/5.0) * 4095;
const int AIR_MIN_VAL = (0.8/5.0) * 4095;
const int AIR_MAX_VAL = (4.0/5.0) * 4095;
const int AIR_MAX_SCS = (4.6/5.0) * 4095;

void readAir() {
  airLastSampleTime = millis();
  // Read raw sensor value
  int airRawFront = readChip(false, CHIP_SELECT_1);
  int airRawRear = readChip(true, CHIP_SELECT_1);
  // Remove oldest sample from sum
  airSumFront -= airSamplesFront[airSampleIndex];
  airSumRear -= airSamplesRear[airSampleIndex];
  // Add new sample
  airSamplesFront[airSampleIndex] = airRawFront;
  airSamplesRear[airSampleIndex] = airRawRear;
  airSumFront += airRawFront;
  airSumRear += airRawRear;
  // Advance buffer index
  airSampleIndex = (airSampleIndex + 1) % AIR_SAMPLE_SIZE;
  // Compute filtered value
  airValFront = airSumFront / AIR_SAMPLE_SIZE;
  airValRear = airSumRear / AIR_SAMPLE_SIZE;
}

void checkAirSCS() {
  airErrorFlagFront = false;
  if (airValFront >= AIR_MAX_SCS) airErrorFlagFront = true;
  if (airValFront >= AIR_MAX_VAL) airValFront = AIR_MAX_VAL;
  if (airValFront <= AIR_MIN_SCS) airErrorFlagFront = true;
  if (airValFront <= AIR_MIN_VAL) airValFront = AIR_MIN_VAL;
  
  airErrorFlagRear = false;
  if (airValRear >= AIR_MAX_SCS) airErrorFlagRear = true;
  if (airValRear >= AIR_MAX_VAL) airValRear = AIR_MAX_VAL;
  if (airValRear <= AIR_MIN_SCS) airErrorFlagRear = true;
  if (airValRear <= AIR_MIN_VAL) airValRear = AIR_MIN_VAL;
}

void calculateAirPressure() {
  // Calculate air Pressure
  airPressureFront = ((airValFront - AIR_MIN_VAL) * (5.0/4095)) / 0.32;
  airPressureRear = ((airValRear - AIR_MIN_VAL) * (5.0/4095)) / 0.32;
  // Clamp final result
  airPressureFront = constrain(airPressureFront, 0, 140);
  airPressureRear = constrain(airPressureRear, 0, 140);
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

  if ((now - lastStatusTime >= STATUS_PERIOD)) {
    sendStatus();
    lastStatusTime = now;
    lastCANMessageTime = now;
    return;
  }

  if ((now - lastSensorsTime >= SENSORS_PERIOD)) {
    sendSensors();
    lastSensorsTime = now;
    lastCANMessageTime = now;
    return;
  }
}

void sendStatus() {
  uint8_t data = 0;
  data |= brakeErrorFlagFront;
  data |= brakeErrorFlagRear << 1;
  data |= airErrorFlagFront << 2;
  data |= airErrorFlagRear << 3;
  data |= sdcStatus << 4;
  data |= ebsTestComplete << 5;
  data |= ebsStatus << 6;

  CAN_message_t msg;
  msg.id = CAN_ID_STATUS;
  msg.buf[0] = data;
  msg.len = 1;
  Can0.write(msg);
}

void sendSensors() {
  CAN_message_t msg;
  msg.id = CAN_ID_SENSORS;
  msg.buf[0] = brakePressureFront;
  msg.buf[1] = brakePressureRear;
  msg.buf[2] = airPressureFront;
  msg.buf[3] = airPressureRear;
  msg.len = 4;
  Can0.write(msg);
}

// --- FSM State Functions ---
// Idle
void onEnterIdle() {
  if (LOG_STATE) Serial.println("IDLE");
}

void duringIdle() {
  if (digitalRead(WD_status) == LOW) {
    fsm.trigger(START_HEARTBEAT);
  }
}

// Start HeartBeat
void onEnterStartHeartbeat() {
  if (LOG_STATE) Serial.println("Start Heartbeat");
  runHeartbeat = true;
  // Reset state variables
  delayComplete = false;
  pulseFinished = false;
  // Start delay timer
  resetTimer = millis();
}

void duringStartHeartbeat() {
  // --- Pre-pulse delay ---
  if (!delayComplete) {
    if (millis() - resetTimer >= resetDelay) {
      // Start Pulse
      delayComplete = true;
      digitalWrite(WD_reset, HIGH);
      resetTimer = millis();
    }
  }
  // --- End HIGH pulse ---
  if (delayComplete && !pulseFinished &&
      millis() - resetTimer >= pulseWidth) {
    digitalWrite(WD_reset, LOW);
    resetTimer = millis(); // reuse timer for settle
    pulseFinished = true;
  }
  // --- Wait settle, then transition ---
  if (pulseFinished &&
      millis() - resetTimer >= settleTime) {
    fsm.trigger(VERIFY_HEARTBEAT);
  }
}

// Check Watchdog Status
void onEnterCheckStatus() {
  if (LOG_STATE) Serial.println("Check Watchdog Status");
}

void duringCheckStatus() {
  if (!watchdogTested && digitalRead(WD_status) == HIGH) {
    fsm.trigger(TEST_HEARTBEAT);
  } else if (watchdogTested && digitalRead(WD_status) == HIGH) {
    fsm.trigger(CLOSE_SDC);
  } else {
    fsm.trigger(ERROR);
  }
}

// Test Watchdog
void onEnterTestWatchdog() {
  if (LOG_STATE) Serial.println("Test Heartbeat");
  runHeartbeat = false;
  resetTimer = millis();
}

void duringTestWatchdog() {
  unsigned long watchdogTimeoutDelay = millis() - resetTimer;
  if (digitalRead(WD_status) == LOW) {
    if (LOG_TIMEOUT) {
      Serial.print("Watchdog timeout: ");
      Serial.println(watchdogTimeoutDelay);
    }
    watchdogTested = true;
    fsm.trigger(START_HEARTBEAT);
  }
  // Watchdog Taking too long -> Error
  if (watchdogTimeoutDelay >= MAX_WATCHDOG_DELAY) {
    fsm.trigger(ERROR);
  }
}

// Close SDC
void onEnterCloseSDC() {
  if (LOG_STATE) Serial.println("Close SDC");
  ebsTestComplete = false; // Reset EBS test
  digitalWrite(AS_close_SDC, HIGH);
}

void duringCloseSDC() {
  if (requestedState == REQ_TEST) fsm.trigger(CHECK_PRESSURE);
}

// Check EBS Pressures
void onEnterCheckPressure() {
  if (LOG_STATE) Serial.println("Check Pressure");
}

void duringCheckPressure() {
  if ((brakePressureRear >= BRAKE_THRESHOLD_ENGAGED) && 
      (brakePressureFront >= BRAKE_THRESHOLD_ENGAGED)) {
    fsm.trigger(DEACTIVATE_REAR);
  }
}

// Deactivate Rear EBS
void onEnterDeactivateRear() {
  if (LOG_STATE) Serial.println("Deactivate Rear EBS");
}

void duringDeactivateRear() {
  digitalWrite(EBS_enbl_rear, HIGH); // Deactivate EBS

  // Check no brake pressure
  if (brakePressureRear <= BRAKE_THRESHOLD_DISENGAGED) fsm.trigger(REACTIVATE_REAR); 
}

// Reactivate Rear EBS
void onEnterReactivateRear() {
  if (LOG_STATE) Serial.println("Reactivate Rear EBS");
}

void duringReactivateRear() {
  digitalWrite(EBS_enbl_rear, LOW); // Reactivate EBS

  // Check brakes locked
  if (brakePressureRear >= BRAKE_THRESHOLD_ENGAGED) fsm.trigger(DEACTIVATE_FRONT); 
}

// Deactivate Front EBS
void onEnterDeactivateFront() {
  if (LOG_STATE) Serial.println("Deactivate Front EBS");
}

void duringDeactivateFront() {
  digitalWrite(EBS_enbl_front, HIGH); // Deactivate EBS

  // Check no brake pressure
  if (brakePressureFront <= BRAKE_THRESHOLD_DISENGAGED) fsm.trigger(REACTIVATE_FRONT); 
}

// Reactivate Front EBS
void onEnterReactivateFront() {
  if (LOG_STATE) Serial.println("Reactivate Front EBS");
}

void duringReactivateFront() {
  digitalWrite(EBS_enbl_front, LOW); // Reactivate EBS

  // Check brakes locked
  if (brakePressureFront >= BRAKE_THRESHOLD_ENGAGED) {
    ebsTestComplete = true;
  }
  if (requestedState == REQ_OFF) {
    fsm.trigger(DRIVING);
  }
}

// Deactivate EBS
void onEnterDeactivateEBS() {
  digitalWrite(EBS_enbl_rear, HIGH); // Deactivate EBS
  digitalWrite(EBS_enbl_front, HIGH); // Deactivate EBS
}

// Continuous Monitoring
void onEnterMonitoring() {
  if (LOG_STATE) Serial.println("Monitoring");
}

void duringMonitoring() {
  // if (airPressureFront <= AIR_THRESHOLD_MIN) {
  //   // TODO: Add Error code
  //   if (LOG_STATE) {
  //     Serial.print("Front Air Pressure: ");
  //     Serial.println(airPressureFront);
  //   }
  //   fsm.trigger(ERROR);
  // }
  // if (airPressureRear <= AIR_THRESHOLD_MIN) {
  //   // TODO: Add Error code
  //   if (LOG_STATE) {
  //     Serial.print("Rear Air Pressure: ");
  //     Serial.println(airPressureRear);
  //   }
  //   fsm.trigger(ERROR);
  // }
  // EStop Request
  if (requestedState == REQ_ON) {
    fsm.trigger(ERROR);
  }
}

// Error
void onEnterError() {
  if (LOG_STATE) Serial.println("Error or ESTOP");
  digitalWrite(AS_close_SDC, LOW); // Open SDC
  digitalWrite(EBS_enbl_rear, LOW); // Activate EBS
  digitalWrite(EBS_enbl_front, LOW); // Activate EBS
}

void duringError() {
  // if (requestedState == REQ_OFF) fsm.trigger(RESET);
}

// FSM Transitions Setup
void setupFSM() {
  // Idle
  fsm.add_transition(&stateIdle, &stateStartHeartbeat, START_HEARTBEAT, NULL);
  
  // Start Heartbeat
  fsm.add_transition(&stateStartHeartbeat, &stateCheckStatus, VERIFY_HEARTBEAT, NULL);

  // Check Watchdog Status
  fsm.add_transition(&stateCheckStatus, &stateTestWatchdog, TEST_HEARTBEAT, NULL);
  fsm.add_transition(&stateCheckStatus, &stateCloseSDC, CLOSE_SDC, NULL);
  fsm.add_transition(&stateCheckStatus, &stateError, ERROR, NULL);

  // Test Heartbeat
  fsm.add_transition(&stateTestWatchdog, &stateStartHeartbeat, START_HEARTBEAT, NULL);
 
  // EBS:
  // Check Pressure
  fsm.add_transition(&stateCloseSDC, &stateCheckPressure, CHECK_PRESSURE, NULL);
  // Test Rear
  fsm.add_transition(&stateCheckPressure, &stateDeactivateRear, DEACTIVATE_REAR, NULL);
  fsm.add_transition(&stateDeactivateRear, &stateReactivateRear, REACTIVATE_REAR, NULL);
  // Test Front
  fsm.add_transition(&stateReactivateRear, &stateDeactivateFront, DEACTIVATE_FRONT, NULL);
  fsm.add_transition(&stateDeactivateFront, &stateReactivateFront, REACTIVATE_FRONT, NULL);
  // Continuous Monitoring
  fsm.add_transition(&stateReactivateFront, &stateDeactivateEBS, DRIVING, NULL);
  fsm.add_timed_transition(&stateDeactivateEBS, &stateMonitoring, 2000, NULL);
  fsm.add_transition(&stateMonitoring, &stateError, ERROR, NULL);
  // RESET
  fsm.add_transition(&stateError, &stateCloseSDC, RESET, NULL);
}

void readVCU(const CAN_message_t &msg) {
  if (msg.id == CAN_ID_VCU_STATUS) {
    // TODO: add AS_SDC
    EBS_Request NEW_STATE = static_cast<EBS_Request>(msg.buf[1] & 0x03);
    if (NEW_STATE != requestedState) {
      if (LOG_STATE) {
        Serial.print("EBS Request: ");
        Serial.println(NEW_STATE);
      }
      requestedState = NEW_STATE;
    }
  }
}

void updateBrakeStatus() {
  if (ebsStatus == EBS_ERROR) return;

  if ((brakePressureFront >= BRAKE_THRESHOLD_ENGAGED) ||
      (brakePressureRear >= BRAKE_THRESHOLD_ENGAGED)) {
    ebsStatus = EBS_ON;
  } else {
    ebsStatus = EBS_OFF;
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  // Setup Digital Pins
  pinMode(WD_status, INPUT);
  pinMode(WD_reset, OUTPUT);
  pinMode(AS_close_SDC, OUTPUT);
  pinMode(HEARTBEAT_PIN, OUTPUT);
  pinMode(EBS_enbl_rear, OUTPUT);
  pinMode(EBS_enbl_front, OUTPUT);

  digitalWrite(WD_reset, LOW);
  digitalWrite(AS_close_SDC, LOW);
  digitalWrite(HEARTBEAT_PIN, LOW);
  digitalWrite(EBS_enbl_rear, LOW);
  digitalWrite(EBS_enbl_front, LOW);

  // Setup chip selects
  for (int i = 0; i < NUM_OF_CHIP_SELECTS; i++) {
    pinMode(ALL_CHIP_SELECTS[i], OUTPUT);
    // LOW to select chip
    digitalWrite(ALL_CHIP_SELECTS[i], HIGH);
  }
  // Setup SPI
  SPI.begin();

  // Initialise CAN
  pinMode(CAN_STBY, OUTPUT);
  digitalWrite(CAN_STBY, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(16);
  Can0.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
  Can0.setMBFilter(MB0, CAN_ID_VCU_STATUS);
  Can0.onReceive(MB0, readVCU);
  Can0.mailboxStatus();

  // Setup State Machine
  setupFSM();
}

// Loop
void loop() {
  // Air Pressure
  if (millis() - airLastSampleTime >= AIR_SAMPLE_INTERVAL) {
    airLastSampleTime = millis();
    readAir();
    checkAirSCS();
    calculateAirPressure();
  }

  // Brake Pressure
  if (millis() - brakeLastSampleTime >= BRAKE_SAMPLE_INTERVAL) {
    brakeLastSampleTime = millis();
    readBrakes();
    checkBrakesSCS();
    calculateBrakePressure();
  }

  // AS-SDC Relay
  sdcStatus = digitalRead(SDC_OUT_status);
  // Update EBS Status
  updateBrakeStatus();

  // On VCU
  // 1. get to AI mode
  // 2. wait for brakes engaged
  // 3. Send REQ_TEST
  // 4. Test the EBS works
  // 5. Respond with Test complete flag
  // 6. Enter AS_READY
  // 7. Send REQ_OFF
  // 8. Enter AS_DRIVING

  // TODO: add timeouts for error
  // TODO: ADD timeout for ESTOP
  // TODO: get back to test again

  // Send CAN messages
  processCAN();

  // Run heartbeat for watchdog
  if (runHeartbeat) updateHeartbeat();
  fsm.run_machine();
}