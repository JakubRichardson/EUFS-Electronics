#include <Arduino.h>
#include <FlexCAN_T4.h>

// Log State
const bool LOG_STATE = false;

// CAN IDs
const int BRAKE_LIGHT_ID = 0x41; // APPS-Status
const int EV_STATE_ID = 0x03; // EV-Status
const int AS_STATE_ID = 0x54; // AI-Status

// Pin definitions
const int yellowPin1 = 3;
const int yellowPin2 = 5;
const int yellowPin3 = 8;
const int bluePin1 = 2;
const int bluePin2 = 4;
const int bluePin3 = 7;

const int readySoundPin = 14;
const int emergencySoundPin = 15;

const int brakeLightPin = 9;
const int brakePressureThreshold = 2; // bar

// CAN Bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const int BAUD_RATE = 500000;

// State definitions implemented as enum: B000 = OFF, B001 = ACTIVE, B010 = MANUAL, B111 = CORSA
typedef enum : uint8_t { 
  TS_OFF = 1,
  TS_ACTIVE = 2,
  MANUAL = 3,
  CORSA = 7
} EV_State;
EV_State CURRENT_EV_STATE = TS_OFF;      // Current state

// State definitions implemented as enum: B00000 = OFF, B00001 = READY, B00010 = DRIVING, B00100 = EMERGENCY, B01000 = FINISHED
typedef enum : uint8_t { 
  OFF = 1,
  READY = 2,
  DRIVING = 3,
  EMERGENCY = 4,
  FINISHED = 5,
  MANUAL_DRIVING = 6
} AS_State;
AS_State CURRENT_AS_STATE = OFF;      // Current state

bool CURRENT_BRAKE_LIGHT_STATE = false;

bool readyTrigger = false;
bool emergencyTrigger = false;

// Color definitions
const int BLUE = 1;
const int YELLOW =  0;


// heartBeat variables
uint8_t heartBeatValue = 0;
long lastHeartBeatTime = 0;
const int heartRate = 250; // ms
void heartBeat() {
  if (millis() - lastHeartBeatTime < heartRate) return;
  // TODO: Complete
}

bool ledOn = false;
void resetLEDs() {
  digitalWrite(yellowPin1, LOW);
  digitalWrite(yellowPin2, LOW);
  digitalWrite(yellowPin3, LOW);
  digitalWrite(bluePin1, LOW);
  digitalWrite(bluePin2, LOW);
  digitalWrite(bluePin3, LOW);
  ledOn = false;
}

void continuous(int color) {
  if (color == BLUE) {
    digitalWrite(bluePin1, HIGH);
    digitalWrite(bluePin2, HIGH);
    digitalWrite(bluePin3, HIGH);
    digitalWrite(yellowPin1, LOW);
    digitalWrite(yellowPin2, LOW);
    digitalWrite(yellowPin3, LOW);
  } else if (color == YELLOW) {
    digitalWrite(yellowPin1, HIGH);
    digitalWrite(yellowPin2, HIGH);
    digitalWrite(yellowPin3, HIGH);
    digitalWrite(bluePin1, LOW);
    digitalWrite(bluePin2, LOW);
    digitalWrite(bluePin3, LOW);
  }
}

unsigned long previousBlinkTime = 0;
void blink(int color, uint updateInterval) {
  if (millis() - previousBlinkTime < updateInterval) return;
  
  if(ledOn) {
    resetLEDs();
  } else {
    continuous(color);
    ledOn = true;
  }
  previousBlinkTime = millis();
}

bool soundOn = false;
void resetSound() {
  digitalWrite(readySoundPin, LOW);
  digitalWrite(emergencySoundPin, LOW);
  soundOn = false;
}

unsigned long soundStartTime = 0;
const int CONTINUOUS_SOUND_DURATION = 2000; // ms
void playReady() {
  if (readyTrigger) {
    soundStartTime = millis();
    readyTrigger = false;
  }
  
  if (millis() - soundStartTime >= CONTINUOUS_SOUND_DURATION) {
    digitalWrite(readySoundPin, LOW);
    return;
  }

  digitalWrite(readySoundPin, HIGH);
}

const int EMERGENCY_SOUND_DURATION = 9000; // Emergency AS_STATE duration in milliseconds
const int EMERGENCY_SOUND_CYCLE = 500;
unsigned long previousSoundTime = 0;
void playEmergency() {
  if (emergencyTrigger) {
    soundStartTime = millis();
    emergencyTrigger = false;
  }
  
  if (millis() - soundStartTime >= EMERGENCY_SOUND_DURATION) {
    digitalWrite(emergencySoundPin, LOW);
    return;
  }

  if (millis() - previousSoundTime < EMERGENCY_SOUND_CYCLE) return;
  
  if(soundOn) {
    digitalWrite(emergencySoundPin, LOW);
    soundOn = false;
  } else {
    digitalWrite(emergencySoundPin, HIGH);
    soundOn = true;
  }
  previousSoundTime = millis();
}

void brakeLight() {
  digitalWrite(brakeLightPin, CURRENT_BRAKE_LIGHT_STATE ? HIGH : LOW);
}

void updateBrakeLight(const CAN_message_t &msg) {
  if (msg.id == BRAKE_LIGHT_ID) {
    uint16_t frontPressure = msg.buf[0] | (msg.buf[1] << 8);
    if(frontPressure > brakePressureThreshold) {
      CURRENT_BRAKE_LIGHT_STATE = true;
    } else {
      CURRENT_BRAKE_LIGHT_STATE = false;
    }
  }
}

void updateEvState(const CAN_message_t &msg) {
  if (msg.id == EV_STATE_ID) {
    EV_State NEW_EV_STATE = static_cast<EV_State>(msg.buf[0] & 0x0F);
    if (NEW_EV_STATE != CURRENT_EV_STATE) {
      if (LOG_STATE) {
        Serial.println(NEW_EV_STATE);
      }
      CURRENT_EV_STATE = NEW_EV_STATE;

      if (CURRENT_EV_STATE == MANUAL) {
        readyTrigger = true;
      }
    }
  }
}

void updateAsState(const CAN_message_t &msg) {
  if (msg.id == AS_STATE_ID) {
    AS_State NEW_AS_STATE = static_cast<AS_State>(msg.buf[0] & 0x0F);
    if (NEW_AS_STATE != CURRENT_AS_STATE) {
      if (LOG_STATE) {
        Serial.println(NEW_AS_STATE);
      }
      CURRENT_AS_STATE = NEW_AS_STATE;

      if (CURRENT_AS_STATE == READY) {
        readyTrigger = true;
      } else if (CURRENT_AS_STATE == EMERGENCY) {
        emergencyTrigger = true;
      }
    }
  }
}

void setup() {
  pinMode(yellowPin1, OUTPUT);
  pinMode(yellowPin2, OUTPUT);
  pinMode(yellowPin3, OUTPUT);

  pinMode(bluePin1, OUTPUT);
  pinMode(bluePin2, OUTPUT);
  pinMode(bluePin3, OUTPUT);

  pinMode(emergencySoundPin, OUTPUT);
  pinMode(readySoundPin, OUTPUT);

  pinMode(brakeLightPin, OUTPUT);

  CURRENT_AS_STATE = OFF;
  CURRENT_EV_STATE = TS_OFF;

  Serial.begin(9600);

  // Initialise CAN
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(3);
  Can0.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
  Can0.setMBFilter(MB0, BRAKE_LIGHT_ID);
  Can0.onReceive(MB0, updateBrakeLight);
  Can0.setMBFilter(MB1, EV_STATE_ID);
  Can0.onReceive(MB1, updateEvState);
  Can0.setMBFilter(MB2, AS_STATE_ID);
  Can0.onReceive(MB2, updateAsState);
  Can0.mailboxStatus();
}

void loop() {
  heartBeat();
  brakeLight();
  switch (CURRENT_EV_STATE) {
    case MANUAL:
      playReady();
      break;
    default:
      resetSound();
  }
  switch (CURRENT_AS_STATE) {
    case OFF:
      resetLEDs();
      resetSound();
      break;
    case READY:
      continuous(YELLOW);
      playReady();
      break;
    case DRIVING:
      blink(YELLOW, 500); 
      resetSound();
      break;
    case EMERGENCY:
      blink(BLUE, 500);
      playEmergency(); 
      break;
    case FINISHED:
      continuous(BLUE);
      resetSound();
      break;
    default:
      resetLEDs();
      resetSound();
  }
}

