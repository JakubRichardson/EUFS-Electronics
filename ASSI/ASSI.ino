#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN IDs
const int stateID = 0x54;

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

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const int BAUD_RATE = 500000;

// State definitions implemented as enum: B00000 = OFF, B00001 = READY, B00010 = DRIVING, B00100 = EMERGENCY, B01000 = FINISHED
typedef enum : uint8_t { 
  OFF = 1,
  READY = 2,
  DRIVING = 3,
  EMERGENCY = 4,
  FINISHED = 5,
  MANUAL = 6
} State;

State CURRENT_STATE = OFF;      // Current state
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
    
  // lastHeartBeatTime = millis();
  // heartBeatValue = !heartBeatValue;
  // digitalWrite(heartBeatPin, heartBeatValue);
  // byte data[sizeof(heartBeatValue)] = {heartBeatValue};
  // CAN.MCP_CAN::sendMsgBuf(HEARTBEAT_CAN_ID, Standard_Frame, 1, data);

  // Can0.send
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

void updateState(const CAN_message_t &msg) {
  if (msg.id == stateID) {
    State NEW_STATE = static_cast<State>(msg.buf[1] & 0x0F);
    if (NEW_STATE != CURRENT_STATE) {
      CURRENT_STATE = NEW_STATE;

      if (CURRENT_STATE == READY) {
        readyTrigger = true;
      } else if (CURRENT_STATE == EMERGENCY) {
        emergencyTrigger = true;
      }
    }

    CURRENT_BRAKE_LIGHT_STATE = (msg.buf[0] & 0x02) > 0;
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

  CURRENT_STATE = OFF;

  Serial.begin(9600);

  // Initialise CAN
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(updateState);
  Can0.mailboxStatus();

  // Can0.setMBFilter(MB0, 0x53);
  // Can0.setMBFilter(MB1, 0x53);
  // Can0.setMBFilter(MB2, 0x53);
  // Can0.setMBFilter(MB3, 0x53);
  // Can0.setMBFilter(MB4, 0x53);
  // Can0.setMBFilter(MB5, 0x53);
  // Can0.setMBFilter(MB6, 0x53);
  // Can0.setMBFilter(MB7, 0x53);
  // Can0.setMBFilter(MB8, 0x53);
  // Can0.setMBFilter(MB9, 0x53);
  // Can0.setMBFilter(MB10, 0x53);
  // Can0.setMBFilter(MB11, 0x53);
  // Can0.setMBFilter(MB12, 0x53);
  // Can0.setMBFilter(MB13, 0x53);
  // Can0.setMBFilter(MB14, 0x53);
  // Can0.setMBFilter(MB15, 0x53);
}

void loop() {
  heartBeat();
  brakeLight();
  switch (CURRENT_STATE) {
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

