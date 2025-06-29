#include <SPI.h>
#include <math.h>
#include <Arduino_CAN.h>
#include <EEPROM.h>

// IMPORTANT: APPS1 SENSOR ADC OUTPUT MUST INCREASE WHEN PEDAL IS PRESSED, APPS2 SENSOR ADC OUTPUT MUST DECREASE WHEN PEDAL IS PRESSED

// ***CALIBRATION VARIABLES***
bool calibrated = true;   // CHANGE TO FALSE TO CALIBRATE, THEN CHANGE TO TRUE AFTER CALIBRATION
float bufferFactor = 0.1; // Adjusts the pedal buffer

// Global Variables
static uint32_t const CAN_ID_TORQUE = 0x201; // For inverter to read toqrue
static uint32_t const CAN_ID_STATUS = 0x41;  // For status messages

const int pressedApps1addr = 0; // EEPROM Addresses For Calibration Variables
const int pressedApps2addr = 1;
const int depressedApps1addr = 2;
const int depressedApps2addr = 3;

const int apps1Pin = A0; // (ADC09/A0/P014/D0) Analog Pins
const int apps2Pin = A1; // (ADC00/A1/P000/D1)
const int bps1Pin = A2;  // (ADC01/A2/P001/D2)
const int bps2Pin = A3;  // (ADC02/A3/P002/D3)

float apps1 = 0; // Float Percentage (0-100) % for APPS output
float apps2 = 0;
int appsOutput; // Int Apps Output % for CAN transmission

int rawBps1;
int rawBps2;

float bps1; // Float BPS Sensors in bar
float bps2;

int bps1Output; // Int BPS Sensors for CAN transmission
int bps2Output;

float minBps = 0; // Plausible range of BPS Sensors in V (3.3V - 0.5 V offset)
float maxBps = 2.8;

int depressedApps1Cal; // depressed pedal ADC value of APPS input, set during calibration
int depressedApps2Cal;

int pressedApps1Cal; // pressed ADC value of APPS input, set during calibration
int pressedApps2Cal;

float offset = 0.5; // BPS Sensor Offset in V

bool appsFlag = 1; // 1 normal, 0 if implausability or sensor value overlap
bool bps1Flag = 1; // 1 normal, 0 if outside sensor range
bool bps2Flag = 1;

int apps1FilterSum = 0; // Average of 92 APPS sensor Samples, sampled at 92Hz
int apps2FilterSum = 0;
int bps1FilterSum = 0;
int bps2FilterSum = 0;

int filteredApps1; // Post-Filter APPS ADC Readings
int filteredApps2;

int i; // For loop index

void setup()
{

  analogReadResolution(10); // Ten bit ADC

  Serial.begin(115200); // Begin serial bus, infinite loop if error
  // while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_500k)) // Begin CAN bus, send error message and infinite loop if error
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;)
    {
    }
  }
}

void loop()
{

  if (calibrated == false)
  { // Run calibration only once if calibration boolean is false

    Serial.println("Release the Acceleration Pedal and Then Press Any Key:");
    // wait for incoming serial data: Then calibrate APPS senors
    waitForSerial();
    depressedApps1Cal = analogRead(apps1Pin) / 5; // Divide by 5 in order to fit in one EEPROM address
    depressedApps2Cal = analogRead(apps2Pin) / 5;

    EEPROM.update(depressedApps1addr, depressedApps1Cal);
    EEPROM.update(depressedApps2addr, depressedApps2Cal);

    Serial.println("Fully Press the Acceleration Pedal and Then Press Any Key:");
    waitForSerial();

    pressedApps1Cal = analogRead(apps1Pin) / 5; // Divide by 5 in order to fit in one EEPROM address
    pressedApps2Cal = analogRead(apps2Pin) / 5;

    EEPROM.update(pressedApps2addr, pressedApps2Cal);
    EEPROM.update(pressedApps1addr, pressedApps1Cal);

    calibrated = true; // Reset calibration boolean so that calibration only runs once
  }

  float depressedApps1 = EEPROM.read(depressedApps1addr) * 5; // Transform back to original analog readings
  float depressedApps2 = EEPROM.read(depressedApps2addr) * 5;
  float pressedApps1 = EEPROM.read(pressedApps1addr) * 5;
  float pressedApps2 = EEPROM.read(pressedApps2addr) * 5;

  float appsBuffer = bufferFactor * (((pressedApps1 - depressedApps1) / 2) + ((depressedApps2 - pressedApps2) / 2)); // Buffer set to a percentage of the average range of ADC values

  for (i = 0; i < 92; i++)
  { // Take 92 samples of raw ADC values and take sum
    apps1FilterSum += analogRead(apps1Pin);
    apps2FilterSum += analogRead(apps2Pin);
    bps1FilterSum += analogRead(bps1Pin);
    bps2FilterSum += analogRead(bps2Pin);
    delay(1);
  }

  filteredApps1 = apps1FilterSum / 92;
  filteredApps2 = apps2FilterSum / 92;
  rawBps1 = bps1FilterSum / 92;
  rawBps2 = bps2FilterSum / 92;

  bps1 = rawBps1 * (3.3 / 1023.0); // ADC ---> Voltage
  bps2 = rawBps2 * (3.3 / 1023.0);

  bps1Output = bps1;
  bps2Output = bps2;

  apps1FilterSum = 0; // Reset sum variables
  apps2FilterSum = 0;
  bps1FilterSum = 0;
  bps2FilterSum = 0;

  // APPS 1 Checks
  appsFlag = 1; // Reset Apps Flag
  if (filteredApps1 < (depressedApps1 + appsBuffer) && filteredApps1 > (depressedApps1 - appsBuffer))
  { // Depressed Buffer Zone
    apps1 = 0;
  }
  else if (filteredApps1 > pressedApps1 && filteredApps1 < (pressedApps1 + appsBuffer))
  { // Pressed Buffer Zone
    apps1 = 100;
  }
  else if (filteredApps1 >= depressedApps1 && filteredApps1 <= pressedApps1)
  { // Linear Zone
    apps1 = ((filteredApps1 - depressedApps1) / (pressedApps1 - depressedApps1)) * 100;
  }
  else
  { // Implausible Zones
    appsFlag = 0;
    apps1 = 0;
    Serial.println("ERROR: APPS 1 OUT OF RANGE");
  }

  // APPS 2 Checks
  if (filteredApps2 > (depressedApps2 - appsBuffer) && filteredApps2 < (depressedApps2 + appsBuffer))
  { // Depressed Buffer Zone
    apps2 = 0;
  }
  else if (filteredApps2 < pressedApps2 && filteredApps2 < (pressedApps2 - appsBuffer))
  { // Pressed Buffer Zone
    apps2 = 100;
  }
  else if (filteredApps2 <= depressedApps2 && filteredApps2 >= pressedApps2)
  { // Linear Zone
    apps2 = ((depressedApps2 - filteredApps2) / (depressedApps2 - pressedApps2)) * 100;
  }
  else
  { // Implausible Zones
    appsFlag = 0;
    apps2 = 0;
    Serial.println("ERROR: APPS 2 OUT OF RANGE");
  }
  // APPS Overlapping check
  if (filteredApps1 <= filteredApps2)
  {
    appsFlag = 0;
    appsOutput = 0;
    Serial.println("ERROR: APPS OVERLAPPING");
  }
  else
  {
    appsOutput = apps1;
  }

  // Implausibility check
  if (abs(apps1 - apps2) >= 10)
  {
    appsFlag = 0;
    appsOutput = 0;
    Serial.println("ERROR: APPS IMPLAUSIBILITY");
  }
  else
  {
    appsOutput = apps1;
  }

  // BPS Checks
  bps1Flag = 1; // Reset BPS Flags
  bps2Flag = 1;

  if (bps1 < minBps || bps1 > maxBps)
  {
    bps1Flag = 0;
    Serial.println("ERROR: BPS1 OUT OF PLAUSIBLE RANGE");
  }
  if (bps2 < minBps || bps2 > maxBps)
  {
    bps2Flag = 0;
    Serial.println("ERROR: BPS2 OUT OF PLAUSIBLE RANGE");
  }

  bps1 = ((bps1 - offset) * 1000.0) / 15.38; // bar... subtracts 0.5V sensor offset ---> 1000 mV/V ---> 15.38 mV/bar
  bps2 = ((bps2 - offset) * 1000.0) / 15.38;

  int16_t torqueCommand = (appsOutput / 100.0) * -3000; // scale 0-100% to 0–3000
  uint8_t torqueLow = torqueCommand & 0xFF;             // Lower byte
  uint8_t torqueHigh = (torqueCommand >> 8) & 0xFF;     // Higher byte

  uint8_t msg_data_torque[] = {0x90, torqueLow, torqueHigh}; // 0x90 for torque mode multiplex on the inverter, if you use 0x31 it will send a speed command.
  uint8_t msg_data_status[] = {appsFlag, bps1Flag, bps2Flag, bps1Output, bps2Output};
  CanMsg const msg(CanStandardId(CAN_ID_TORQUE), sizeof(msg_data_torque), msg_data_torque);
  CanMsg const statusMsg(CanStandardId(CAN_ID_STATUS), sizeof(msg_data_status), msg_data_status);

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
  Serial.println("Torque command:");
  Serial.println(torqueCommand);
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
  int const rc = CAN.write(msg);
  delay(1);
  int const rc1 = CAN.write(statusMsg);

  if (rc < 0 || rc1 < 0)
  {
    Serial.print("CAN.write(...) failed with error code ");
    Serial.println(rc);
  }
}

void waitForSerial()
{
  while (!Serial.available())
  {
  }
  Serial.println(Serial.read());
}
