#include <SPI.h>
#include <math.h>
#include <Arduino_CAN.h>
#include <EEPROM.h>

// IMPORTANT: APPS1 SENSOR ADC OUTPUT MUST INCREASE WHEN PEDAL IS PRESSED, APPS2 SENSOR ADC OUTPUT MUST DECREASE WHEN PEDAL IS PRESSED

// ***CALIBRATION VARIABLES***
bool calibrated = true; // CHANGE TO FALSE TO CALIBRATE, THEN CHANGE TO TRUE AFTER CALIBRATION
float bufferFactor = 0.1; //Adjusts the pedal buffer

// Global Variables
static uint32_t const CAN_ID_TORQUE = 0x201; //For inverter to read toqrue
static uint32_t const CAN_ID_STATUS = 0x41; //For status messages
static uint32_t const CAN_ID_VCU_STATUS = 0x3; //VCU Status
static uint32_t const CAN_FILTER_MASK_STANDARD = 0x1FFC0000;

////////////////////////////////////////////////////////////////////////////

const int pressedApps1addr = 0; // EEPROM Addresses For Calibration Variables
const int pressedApps2addr = 1;
const int depressedApps1addr = 2;
const int depressedApps2addr = 3;


int depressedApps1Cal; // depressed pedal ADC value of APPS input, set during calibration
int depressedApps2Cal;

int pressedApps1Cal; // pressed ADC value of APPS input, set during calibration
int pressedApps2Cal;

float depressedApps1;
float depressedApps2;
float pressedApps1;
float pressedApps2;


////////////////////////////////////////////////////////////////////////////


const int apps1Pin = A0;  // (ADC09/A0/P014/D0) Analog Pins
const int apps2Pin = A1;  // (ADC00/A1/P000/D1)
const int bps1Pin = A2;  // (ADC01/A2/P001/D2)
const int bps2Pin = A3;  // (ADC02/A3/P002/D3)

float apps1 = 0;  //Float Percentage (0-100) % for APPS output
float apps2 = 0;
int appsOutput; //Int Apps Output % for CAN transmission
float appsBuffer = 0;

////////////////////////////////////////////////////////////////////////////


int rawBps1;

int16_t bps1 = 0;   //Float BPS Sensors in bar

int bps1Output; // Int BPS Sensors for CAN transmission

float minBps = 0; // Plausible range of BPS Sensors in V (3.3V - 0.5 V offset)
float maxBps = 2.8;


float offset = 0.5; // BPS Sensor Offset in V

bool appsFlag = 0; // 1 normal, 0 if implausability or sensor value overlap
bool bps1Flag = 0; // 1 normal, 0 if outside sensor range
bool pedalActive = 0;

int apps1FilterSum = 0; // Average of 92 APPS sensor Samples, sampled at 92Hz
int apps2FilterSum = 0;
int bps1FilterSum = 0;

int filteredApps1; // Post-Filter APPS ADC Readings
int filteredApps2;
int filteredBps1;


enum VCUState {
  TS_OFF = 1,
  TS_ACTIVE = 2,
  MANUAL_DRIVING = 3,
  CORSA_MODE = 7
};

VCUState currentState = TS_OFF;
unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatTimeout = 100; // ms, you can increase to 500ms or more

int16_t torqueCommand = 0;
uint8_t torqueLow = 0;
uint8_t bps1Low = 0;
uint8_t torqueHigh = 0;
uint8_t bps1High = 0;
uint8_t statusFlags = 0;


void setup() {
 
  analogReadResolution(10); //Ten bit ADC
 
  Serial.begin(115200); //Begin serial bus, infinite loop if error
  //while (!Serial) { }
  CAN.setFilterMask_Standard(CAN_FILTER_MASK_STANDARD);

  for (int mailbox = 0; mailbox < R7FA4M1_CAN::CAN_MAX_NO_STANDARD_MAILBOXES; mailbox++)
  {
    CAN.setFilterId_Standard(mailbox, CAN_ID_VCU_STATUS);
  }

  if (!CAN.begin(CanBitRate::BR_500k)) //Begin CAN bus, send error message and infinite loop if error
  {
    Serial.println("CAN.begin(...) failed.");
  } 
}

void loop() {

    if(calibrated == false){ //Run calibration only once if calibration boolean is false

      Serial.println("Release the Acceleration Pedal and Then Press Any Key:");
      // wait for incoming serial data: Then calibrate APPS senors
      waitForSerial();
//      depressedApps1Cal = analogRead(apps1Pin) / 5; // Divide by 5 in order to fit in one EEPROM address
//      depressedApps2Cal = analogRead(apps2Pin) / 5;
//
//      EEPROM.update(depressedApps1addr, depressedApps1Cal);
//      EEPROM.update(depressedApps2addr, depressedApps2Cal);

      EEPROM.put(depressedApps1addr, analogRead(apps1Pin));
      EEPROM.put(depressedApps2addr, analogRead(apps2Pin));

      Serial.println("Fully Press the Acceleration Pedal and Then Press Any Key:");
      waitForSerial();
    
//      pressedApps1Cal = analogRead(apps1Pin) / 5; // Divide by 5 in order to fit in one EEPROM address
//      pressedApps2Cal = analogRead(apps2Pin) / 5;
//
//      EEPROM.update(pressedApps2addr, pressedApps2Cal);
//      EEPROM.update(pressedApps1addr, pressedApps1Cal);
      EEPROM.put(pressedApps2addr, analogRead(apps2Pin));
      EEPROM.put(pressedApps1addr, analogRead(apps1Pin));

      calibrated = true; // Reset calibration boolean so that calibration only runs once

  }
  getVCUStatus();             // Read and update state
  movingAverageSensor();      // Sample ADC and calculate rolling average
  getAppsBuffer();            // Calculate buffer from EEPROM (could be moved to setup if fixed)

  if (currentState == MANUAL_DRIVING) {
    calculateAppsOutput();
    appsImplausibilityCheck();
    calculateTorqueCommand();
  } else {
    appsOutput = 0;
    torqueCommand = 0;
    pedalActive = 0; // disallow torque transmission outside manual mode
  }

  calculateBsp();             // Convert filtered voltage to bar
  checkBpsPlausibility();     // Check if bar is plausible
  sendCanMessages();          // Always send torque (0 if not allowed) + status

  // Optional: Print debug
  Serial.print("Current State: ");
  Serial.println(currentState);
  

  // Serial Messages
  Serial.println("Apps Flag:");
  Serial.println(appsFlag);
  Serial.println(" ");
//  Serial.println("BPS1 Flag:");
//  Serial.println(bps1Flag);
//  Serial.println(" ");
//  Serial.println("BPS2 Flag:");
//  Serial.println(bps2Flag);
//  Serial.println(" ");
  Serial.println("APPS Output Percentage:");
  Serial.println(appsOutput);
  Serial.println(" ");
//  Serial.println("Brake Pressure 1 (bar): ");
//  Serial.println(bps1Output);
//  Serial.println(" ");
//  Serial.println("Brake Pressure 2 (bar): ");
//  Serial.println(bps2Output);
//  Serial.println(" ");

  /* Transmit the CAN message, capture and display an
   * error core in case of failure.
   */
//  int const rc = CAN.write(msg);
//  delay(1);
//  int const rc1 = CAN.write(statusMsg);
//
//  
//  if (rc < 0 || rc1 < 0)
//  {
//    Serial.print  ("CAN.write(...) failed with error code ");
//    Serial.println(rc);
//  }

}

const int sampleSize = 100;
int apps1Samples[sampleSize] = {0};
int apps2Samples[sampleSize] = {0};
int bps1Samples[sampleSize] = {0};

int sampleIndex = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 1; // in milliseconds


void movingAverageSensor() {
  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    // Read new samples
    apps1Samples[sampleIndex] = analogRead(apps1Pin);
    apps2Samples[sampleIndex] = analogRead(apps2Pin);
    bps1Samples[sampleIndex] = analogRead(bps1Pin);

    // Advance the circular buffer index
    sampleIndex = (sampleIndex + 1) % sampleSize;

    // Calculate rolling averages
    long sumApps1 = 0;
    long sumApps2 = 0;
    long sumBps1 = 0;

    for (int i = 0; i < sampleSize; i++) {
      sumApps1 += apps1Samples[i];
      sumApps2 += apps2Samples[i];
      sumBps1  += bps1Samples[i];
    }

    filteredApps1 = sumApps1 / sampleSize;
    filteredApps2 = sumApps2 / sampleSize;
    filteredBps1  = sumBps1  / sampleSize;
  }
}

void calculateAppsOutput(){
  //APPS 1 Checks
  appsFlag = 0; //Reset Apps Flag
  pedalActive = 1;
  if(filteredApps1 < (depressedApps1 + appsBuffer) && filteredApps1 > (depressedApps1 - appsBuffer)){ //Depressed Buffer Zone
      apps1 = 0;     
  }
  else if(filteredApps1 > pressedApps1 && filteredApps1 < (pressedApps1 + appsBuffer)){ // Pressed Buffer Zone
    apps1 = 100;
      }
  else if(filteredApps1 >= depressedApps1 && filteredApps1 <= pressedApps1){ // Linear Zone
      apps1 = ((filteredApps1 - depressedApps1) / (pressedApps1 - depressedApps1)) * 100;
  }
  else{ // Implausible Zones
    appsFlag = 1;
    pedalActive = 0;
    apps1 = 0;
    Serial.println("ERROR: APPS 1 OUT OF RANGE");
  }

  //APPS 2 Checks
  if(filteredApps2 > (depressedApps2 - appsBuffer) && filteredApps2 < (depressedApps2 + appsBuffer)){ // Depressed Buffer Zone
      apps2 = 0;     
  }
  else if(filteredApps2 < pressedApps2 && filteredApps2 < (pressedApps2 - appsBuffer)){ // Pressed Buffer Zone
    apps2 = 100;
      }
  else if(filteredApps2 <= depressedApps2 && filteredApps2 >= pressedApps2){ // Linear Zone
      apps2 = ((depressedApps2 - filteredApps2) / (depressedApps2 - pressedApps2)) * 100;
  }
  else{ // Implausible Zones
    appsFlag = 1;
    pedalActive = 0;
    apps2 = 0;
    Serial.println("ERROR: APPS 2 OUT OF RANGE");
  } 
  
  appsOutput = apps1;
  
}

void appsImplausibilityCheck(){
  if (abs(apps1 - apps2) >= 10){
    appsFlag = 1;
    pedalActive = 0;
    appsOutput = 0;
    Serial.println("ERROR: APPS IMPLAUSIBILITY");
  }
  else{
    appsOutput = apps1;
  }
}



void calculateTorqueCommand() {
  if (currentState == MANUAL_DRIVING) {
    torqueCommand = ((float)appsOutput / 100.0) * -3000;
  } else {
    torqueCommand = 0;
    pedalActive = 0;
  }

  torqueLow = torqueCommand & 0xFF;
  torqueHigh = (torqueCommand >> 8) & 0xFF;
}


void getAppsBuffer(){
 
  EEPROM.get(depressedApps1addr, depressedApps1);
  EEPROM.get(depressedApps2addr, depressedApps2);
  EEPROM.get(pressedApps1addr, pressedApps1);
  EEPROM.get(pressedApps2addr, pressedApps2);

  float appsBuffer = bufferFactor * ( ((pressedApps1 - depressedApps1) / 2) + ((depressedApps2 - pressedApps2) / 2) ); // Buffer set to a percentage of the average range of ADC values
}

void checkBpsPlausibility(){
  //BPS Checks
  bps1Flag = 0; //Reset BPS Flags
//  bps2Flag = 1;

  if (bps1 < minBps || bps1 > maxBps){
    bps1Flag = 1;
    Serial.println("ERROR: BPS1 OUT OF PLAUSIBLE RANGE");
    }
//  if (bps2 < minBps || bps2 > maxBps){
//    bps2Flag = 1;
//    Serial.println("ERROR: BPS2 OUT OF PLAUSIBLE RANGE");
//    }
}

void calculateBsp(){
  bps1 = ((bps1 - offset) * 1000.0) / 15.38; // bar... subtracts 0.5V sensor offset ---> 1000 mV/V ---> 15.38 mV/bar
//  bps2 = ((bps2 - offset) * 1000.0) / 15.38;
  bps1Low = bps1 & 0xFF;
  bps1High = (bps1 >> 8) & 0xFF;
}

void sendCanMessages(){
  statusFlags |= (bps1Flag    & 0x01) << 0;
  statusFlags |= (0           & 0x01) << 1;
  statusFlags |= (appsFlag    & 0x01) << 2;
  statusFlags |= (pedalActive & 0x01) << 3;
  uint8_t msg_data_torque[] = {0x90, torqueLow, torqueHigh}; //0x90 for torque mode multiplex on the inverter, if you use 0x31 it will send a speed command.
  uint8_t msg_data_status[] = {bps1Low, bps1High, 0, 0, appsOutput, statusFlags};
  CanMsg const msg(CanStandardId(CAN_ID_TORQUE), sizeof(msg_data_torque), msg_data_torque);
  CanMsg const statusMsg(CanStandardId(CAN_ID_STATUS), sizeof(msg_data_status), msg_data_status);

  int const rc = CAN.write(msg);
  delay(1); // DOUBLE CHECK THE NEED FOR THIS DELAY
  int const rc1 = CAN.write(statusMsg);

  
  if (rc < 0 || rc1 < 0)
  {
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.println(rc);
  }
  
}


void getVCUStatus() {
  if (CAN.available()) {
    CanMsg const msg = CAN.read();

    if (msg.id == CAN_ID_VCU_STATUS) {
      currentState = static_cast<VCUState>(msg.data[0]);
      lastHeartbeatTime = millis(); // reset heartbeat timeout
    }
  }

  // If no message in a while, fail safe to TS_OFF
  if (millis() - lastHeartbeatTime > heartbeatTimeout) {
    currentState = TS_OFF;
  }
}


// IMPORTANT TO READ THE BUTTON PRESS DURING CALIBRATION

void waitForSerial(){
  while (!Serial.available()) {
  }
  Serial.println(Serial.read());
}
