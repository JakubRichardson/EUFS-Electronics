// Pin definitions
const int HEARTBEAT_PIN = 2;   // Heartbeat output
const int WD_reset  = 3;       // Digital output
const int WD_status = 4;       // Digital input
const int AS_close_SDC = 5;    // Digital output

// Heartbeat timing (10 Hz → toggle every 50 ms)
unsigned long lastHeartbeatToggle = 0;
const unsigned long heartbeatInterval = 20; // ms
bool heartbeatState = false;

unsigned long lastStatusRead = 0;
int stepCount = 0;
bool SDC_state = false;

// --- Heartbeat update (non-blocking) ---
void updateHeartbeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatToggle >= heartbeatInterval) {
    lastHeartbeatToggle = currentMillis;
    heartbeatState = !heartbeatState;
    digitalWrite(HEARTBEAT_PIN, heartbeatState);
  }
}

// --- Status Read (non-blocking) ---
void readStatus() {
  if (millis() - lastStatusRead >= 2000) {
    Serial.print("WD_status: ");
    Serial.print(digitalRead(WD_status));
    Serial.print(", AS_close_SDC: ");
    Serial.print(SDC_state);
    Serial.print(", Step: ");
    Serial.println(stepCount);
    
    lastStatusRead = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(WD_status, INPUT);
  pinMode(WD_reset, OUTPUT);
  pinMode(AS_close_SDC, OUTPUT);
  pinMode(HEARTBEAT_PIN, OUTPUT);

  digitalWrite(WD_reset, LOW);
  digitalWrite(AS_close_SDC, LOW);
  digitalWrite(HEARTBEAT_PIN, LOW);
}

void loop() {
  digitalWrite(AS_close_SDC, SDC_state);
  readStatus();
  if (stepCount != 0) {
    updateHeartbeat();
  }
  if (stepCount == 2) {
    digitalWrite(WD_reset, HIGH);
  }
  if (stepCount == 3) {
    digitalWrite(WD_reset, LOW);
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0') {
      stepCount = ((stepCount + 1) % 4);
    } else if (c == '1') {
      SDC_state = !SDC_state;
    }
  }
}