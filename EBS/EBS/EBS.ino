#include <Fsm.h>

// Logging
bool LOG_STATE = true;
bool LOG_TIMEOUT = true;

// Pin definitions
const int HEARTBEAT_PIN = 2;
const int WD_reset  = 3;
const int WD_status = 4;
const int AS_close_SDC = 5;

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
#define START_HEARTBEAT 0
#define VERIFY_HEARTBEAT 1
#define TEST_HEARTBEAT 2
#define CLOSE_SDC 3
#define WD_ERROR 4

// FSM States
State stateIdle(&onEnterIdle, &duringIdle, NULL);
State stateStartHeartbeat(&onEnterStartHeartbeat, &duringStartHeartbeat, NULL);
State stateCheckStatus(&onEnterCheckStatus, &duringCheckStatus, NULL);
State stateTestWatchdog(&onEnterTestWatchdog, &duringTestWatchdog, NULL);
State stateCloseSDC(&onEnterCloseSDC, NULL, NULL);
State stateError(&onEnterError, NULL, NULL);
Fsm fsm(&stateIdle);

// Heartbeat update
void updateHeartbeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatToggle >= heartbeatInterval) {
    lastHeartbeatToggle = currentMillis;
    heartbeatState = !heartbeatState;
    digitalWrite(HEARTBEAT_PIN, heartbeatState);
  }
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
    fsm.trigger(WD_ERROR);
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
  if (watchdogTimeoutDelay <= MAX_WATCHDOG_DELAY) {
    fsm.trigger(WD_ERROR);
  }
}

// Close SDC
void onEnterCloseSDC() {
  if (LOG_STATE) Serial.println("Close SDC");
  digitalWrite(AS_close_SDC, HIGH);
}

// Error
void onEnterError() {
  if (LOG_STATE) Serial.println("Watchdog Error");
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
  fsm.add_transition(&stateCheckStatus, &stateError, WD_ERROR, NULL);

  // Test Heartbeat
  fsm.add_transition(&stateTestWatchdog, &stateStartHeartbeat, START_HEARTBEAT, NULL);
 }

// Setup
void setup() {
  Serial.begin(115200);
  pinMode(WD_status, INPUT);
  pinMode(WD_reset, OUTPUT);
  pinMode(AS_close_SDC, OUTPUT);
  pinMode(HEARTBEAT_PIN, OUTPUT);

  digitalWrite(WD_reset, LOW);
  digitalWrite(AS_close_SDC, LOW);
  digitalWrite(HEARTBEAT_PIN, LOW);

  // Setup State Machine
  setupFSM();
}

// Loop
void loop() {
  if (runHeartbeat) {
    updateHeartbeat();
  }
  fsm.run_machine();
}