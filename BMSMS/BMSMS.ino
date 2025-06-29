#include "SPI.h"
#include <FlexCAN_T4.h>

// CONFIGURATION
// ================

// NOTE: Change for each module, IMPORTANT!!!
const int MODULE_INDEX = 3;

const float FREQUENCY_HZ = 50;
const float FREQUENCY_ADDRESS_CLAIM_HZ = 10;

const bool CHANNEL_1 = true;  // False to read channel 0

const float VDD = 5.0;

const bool LOG_READING = false;
const bool LOG_VOLT = false;
const bool LOG_TEMP = false;
const bool LOG_OUTPUT = false;

// CAN
// ================

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const int BAUD_RATE = 250000;

// https://www.orionbms.com/downloads/misc/thermistor_module_canbus.pdf
const int CAN_ID_ADDRESS_CLAIM = 0x18EEFF80 + MODULE_INDEX;
const int CAN_ID_THERMISTOR_MODULE = 0x1839F380 + MODULE_INDEX;
const int CAN_ID_THERMISTOR_GENERAL = 0x1838F380 + MODULE_INDEX;

// PINS
// ================

// CIPO - D12
// COPI - D11
// SCK  - D13

const int NUM_OF_CHIP_SELECTS = 3;

const int CHIP_SELECT_0 = 10;
const int CHIP_SELECT_1 = 8;
const int CHIP_SELECT_2 = 9;
const int ALL_CHIP_SELECTS[NUM_OF_CHIP_SELECTS] = {
  CHIP_SELECT_0,
  CHIP_SELECT_1,
  CHIP_SELECT_2,
};

const int NUM_THERMISTOR_SELECTS = 6;

// Select thermistor from chip: "CBA" (C is MSB, A is LSB)
const unsigned int NUM_THERMISTOR_SELECT_BITS = 3;
const int THERMISTOR_SELECT_A = 17;
const int THERMISTOR_SELECT_B = 16;
const int THERMISTOR_SELECT_C = 15;
const int THERMISTOR_SELECT_BITS[NUM_THERMISTOR_SELECT_BITS] = {
  THERMISTOR_SELECT_A,
  THERMISTOR_SELECT_B,
  THERMISTOR_SELECT_C
};

const int THERMISTOR_COUNT = NUM_OF_CHIP_SELECTS * NUM_THERMISTOR_SELECTS;

const int THERMISTOR_SELECT_ENABLE = 14;

const int CAN_OUTPUT_PIN = 1; // TX1 = D1

// DATA VARS
// ================

// NOTE: NAN indicates invalid temperature
float temperatures[NUM_OF_CHIP_SELECTS][NUM_THERMISTOR_SELECTS];

float minTemperature, maxTemperature, meanTemperature;
signed char minTemperatureIndex, maxTemperatureIndex;

const float MIN_VALID_VOLTAGE = 0.1;
const float MAX_VALID_VOLTAGE = 4.2;
const float MIN_PERCENT_VALID_TEMPERATURES = 75.0;

const float INVALID_TEMPERATURE = 85.0;

// LOOKUP TABLE
// ================

struct v_to_t_lookup_entry {
  float temp;
  float voltage;
};

const v_to_t_lookup_entry V_TO_T_LOOKUP_TABLE[] = {
  { -40.0, 2.44 },
  { -35.0, 2.42 },
  { -30.0, 2.40 },
  { -25.0, 2.38 },
  { -20.0, 2.35 },
  { -15.0, 2.32 },
  { -10.0, 2.27 },
  {  -5.0, 2.23 },
  {   0.0, 2.17 },
  {   5.0, 2.11 },
  {  10.0, 2.05 },
  {  15.0, 1.99 },
  {  20.0, 1.92 },
  {  25.0, 1.86 },
  {  30.0, 1.80 },
  {  35.0, 1.74 },
  {  40.0, 1.68 },
  {  45.0, 1.63 },
  {  50.0, 1.59 },
  {  55.0, 1.55 },
  {  60.0, 1.51 },
  {  65.0, 1.48 },
  {  70.0, 1.45 },
  {  75.0, 1.43 },
  {  80.0, 1.40 },
  {  85.0, 1.38 },
  {  90.0, 1.37 },
  {  95.0, 1.35 },
  { 100.0, 1.34 },
  { 105.0, 1.33 },
  { 110.0, 1.32 },
  { 115.0, 1.31 },
  { 120.0, 1.30 }
};
const int NUM_V_TO_T_LOOKUP_ENTRIES = sizeof(V_TO_T_LOOKUP_TABLE) / sizeof(V_TO_T_LOOKUP_TABLE[0]);

// CONSTANTS
// ================

const float MAX_FLOAT = 3.4028235E+38;
const float MIN_FLOAT = -3.4028235E+38;

// CODE
// vvvvvvvvvvvvvvvv

int nextTemp;
int nextClaimAddress;

/**
 * @brief Setup the Arduino
 */
void setup() {
  Serial.begin(9600);
  Serial.println("HERE");

  // Setup chip selects
  for (int i = 0; i < NUM_OF_CHIP_SELECTS; i++) {
    pinMode(ALL_CHIP_SELECTS[i], OUTPUT);
    // LOW to select chip
    digitalWrite(ALL_CHIP_SELECTS[i], HIGH);
  }

  // Setup thermistor select
  for (unsigned int i = 0; i < NUM_THERMISTOR_SELECT_BITS; i++) {
    pinMode(THERMISTOR_SELECT_BITS[i], OUTPUT);
    // Select 0
    digitalWrite(THERMISTOR_SELECT_BITS[i], LOW);
  }
  pinMode(THERMISTOR_SELECT_ENABLE, OUTPUT);
  digitalWrite(THERMISTOR_SELECT_ENABLE, LOW);
  
  // Setup SPI
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Initialise CAN
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(BAUD_RATE);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.mailboxStatus();

  nextTemp = millis();
  nextClaimAddress = millis();
}

/**
 * @brief Read the value using SPI
 * 
 * @param chipSelect The chip select
 * @return unsigned int Value read
 */
unsigned int readChip(unsigned int chipSelect) {
  int chipSelectPin = ALL_CHIP_SELECTS[chipSelect];

  // https://github.com/souviksaha97/MCP3202/blob/master/src/MCP3202.cpp#L48
  unsigned int dataIn = 0;
  unsigned int result = 0;
  digitalWrite(chipSelectPin, LOW);
  uint8_t dataOut = 0b00000001;
  dataIn = SPI.transfer(dataOut);
  int dataOutChannel0 = 0b10100000;
  int dataOutChannel1 = 0b11100000;
  dataOut = CHANNEL_1 ? dataOutChannel1 : dataOutChannel0;
  dataIn = SPI.transfer(dataOut);
  result = dataIn & 0x0F;
  dataIn = SPI.transfer(0x00);
  result = result << 8;
  result = result | dataIn;
  //input = input << 1;
  digitalWrite(chipSelectPin, HIGH);
  return result;
}

/**
 * @brief Calculate the temperate from the value read
 * 
 * @param reading The value read
 * @return float The temperature
 */
float calculateTemperature(unsigned int reading) {
  float voltage = (reading * VDD) / 4096.0;

  if (LOG_VOLT) {
    Serial.printf("%fV\t", voltage);
  }

  if (MIN_VALID_VOLTAGE > voltage || voltage > MAX_VALID_VOLTAGE) {
    // NAN indicates invalid temp
    return NAN;
  }

  if (voltage >= V_TO_T_LOOKUP_TABLE[0].voltage) {
    return V_TO_T_LOOKUP_TABLE[0].temp;
  }
  if (voltage <= V_TO_T_LOOKUP_TABLE[NUM_V_TO_T_LOOKUP_ENTRIES - 1].voltage) {
    return V_TO_T_LOOKUP_TABLE[NUM_V_TO_T_LOOKUP_ENTRIES - 1].temp;
  }

  for (int i = 0; i < NUM_V_TO_T_LOOKUP_ENTRIES - 1; i++) {
    // NOTE: Voltage goes down as index goes up (as temp goes up)
    v_to_t_lookup_entry low = V_TO_T_LOOKUP_TABLE[i + 1];
    v_to_t_lookup_entry high = V_TO_T_LOOKUP_TABLE[i];

    if (voltage >= low.voltage && voltage <= high.voltage) {
      float gradient = (high.temp - low.temp) / (high.voltage - low.voltage);
      float intercept = low.temp - (gradient * low.voltage);
      return (voltage * gradient) + intercept;
    }
  }

  Serial.printf("ERROR: Couldn't lookup %f\n", voltage);
  return 0;
}

/**
 * @brief Set the thermistor select values
 * The bits for the thermistor select are set
 * 
 * @param thermistorSelect The thermistor to select
 */
void setTherimistorSelect(unsigned int thermistorSelect) {
  if (thermistorSelect >= NUM_THERMISTOR_SELECTS) {
    Serial.println("ERROR: Thermistor select value too large");
    return;
  }

  for (unsigned int i = 0; i < NUM_THERMISTOR_SELECT_BITS; i++) {
    digitalWrite(THERMISTOR_SELECT_BITS[i], thermistorSelect & 0x01);
    thermistorSelect = thermistorSelect >> 1;
  }
}

/**
 * @brief Read the temperatures from the thermistors in all chips and stor them in the temperatures array
 */
void readTemperatures() {
  // Enable thermistors
  digitalWrite(THERMISTOR_SELECT_ENABLE, HIGH);

  for (unsigned int ts = 0; ts < NUM_THERMISTOR_SELECTS; ts++) {
    setTherimistorSelect(ts);
    // NOTE: 400 seems to be around the minimum required delay
    delayMicroseconds(1000);
    for (unsigned int cs = 0; cs < NUM_OF_CHIP_SELECTS; cs++) {
      unsigned int reading = readChip(cs);
      if (LOG_READING) {
        Serial.printf("%d\t", reading);
      }
      float temperature = calculateTemperature(reading);
      if (LOG_TEMP) {
        Serial.printf("%fC\t", temperature);
      }
      if (LOG_TEMP || LOG_VOLT || LOG_READING) {
        Serial.printf("(%d, %d)\n", cs, ts);
      }

      temperatures[cs][ts] = temperature;
    }
  }

  // Ensure thermistors disabled so they do not heat up
  digitalWrite(THERMISTOR_SELECT_ENABLE, LOW);
}

/**
 * @brief Calculate the statistics (min, max, mean) of the temperatures
 */
void calculateStatistics() {
  minTemperature = MAX_FLOAT;
  maxTemperature = MIN_FLOAT;
  minTemperatureIndex = -1;
  maxTemperatureIndex = -1;

  int sumTemperatures = 0;
  int validThermistors = 0;

  for (int ts = 0; ts < NUM_THERMISTOR_SELECTS; ts++) {
    for (int cs = 0; cs < NUM_OF_CHIP_SELECTS; cs++) {
      float temperature = temperatures[cs][ts];

      // Skip invalid temperatures
      if (isnan(temperature)) {
        continue;
      }

      if (temperature < minTemperature) {
        minTemperature = temperature;
        minTemperatureIndex = cs * NUM_THERMISTOR_SELECTS + ts;
      }
      if (temperature > maxTemperature) {
        maxTemperature = temperature;
        maxTemperatureIndex = cs * NUM_THERMISTOR_SELECTS + ts;
      }

      sumTemperatures += temperature;
      validThermistors++;
    }
  }

  meanTemperature = round((float)sumTemperatures / (float)validThermistors);

  if (THERMISTOR_COUNT * (MIN_PERCENT_VALID_TEMPERATURES / 100) > validThermistors) {
    Serial.println("Number of invalid temperature readings exceeds threshold.");
    minTemperature = INVALID_TEMPERATURE;
    maxTemperature = INVALID_TEMPERATURE;
    minTemperatureIndex = 0;
    maxTemperatureIndex = 0;
    meanTemperature = INVALID_TEMPERATURE;
  }

  if (LOG_OUTPUT) {
    Serial.printf("Mean: %f\nMin: %f (%d)\nMax: %f (%d)\n", meanTemperature, minTemperature, minTemperatureIndex, maxTemperature, maxTemperatureIndex);
  }
}

void sendAddressClaim() {
  CAN_message_t msg;
  msg.id = CAN_ID_ADDRESS_CLAIM;
  msg.flags.extended = true;
  msg.buf[0] = 0xF3;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x80 + MODULE_INDEX;
  msg.buf[3] = 0xF3;
  msg.buf[4] = MODULE_INDEX << 3;
  msg.buf[5] = 0x40;
  msg.buf[6] = 0x1E;
  msg.buf[7] = 0x90;

  Can0.write(msg);
}

void sendThermistorModule() {
  int validThermistors = 0;
  for (int ts = 0; ts < NUM_THERMISTOR_SELECTS; ts++) {
    for (int cs = 0; cs < NUM_OF_CHIP_SELECTS; cs++) {
      float temperature = temperatures[cs][ts];
      if (!isnan(temperature)) {
        validThermistors++;
      }
    }
  }

  bool fault = THERMISTOR_COUNT * (MIN_PERCENT_VALID_TEMPERATURES / 100) > validThermistors;

  CAN_message_t msg;
  msg.id = CAN_ID_THERMISTOR_MODULE;
  msg.flags.extended = true;
  msg.buf[0] = MODULE_INDEX;
  msg.buf[1] = (int8_t) minTemperature;
  msg.buf[2] = (int8_t) maxTemperature;
  msg.buf[3] = (int8_t) meanTemperature;
  msg.buf[4] = validThermistors + (fault ? 0x80 : 0);
  msg.buf[5] = minTemperatureIndex;
  msg.buf[6] = maxTemperatureIndex;

  // Checksum 8-bit (sum of all bytes + 0x39 + length)
  msg.buf[7] = msg.buf[0] + msg.buf[1] + msg.buf[2] + msg.buf[3] + msg.buf[4] + msg.buf[5] + msg.buf[6] +
               0x39 + 8;

  Can0.write(msg);
}

int thermistorID = 0;

void sendThermistorGeneral() {
  int cs = thermistorID / NUM_THERMISTOR_SELECTS;
  int ts = thermistorID % NUM_THERMISTOR_SELECTS;

  CAN_message_t msg;
  msg.id = CAN_ID_THERMISTOR_GENERAL;
  msg.flags.extended = true;
  msg.buf[0] = 0x00;
  msg.buf[1] = thermistorID + MODULE_INDEX * THERMISTOR_COUNT;
  msg.buf[2] = isnan(temperatures[cs][ts]) ? -128 : (int8_t) temperatures[cs][ts];
  msg.buf[3] = thermistorID;
  msg.buf[4] = (int8_t) minTemperature;
  msg.buf[5] = (int8_t) maxTemperature;
  msg.buf[6] = maxTemperatureIndex;
  msg.buf[7] = minTemperatureIndex;

  Can0.write(msg);

  thermistorID = (thermistorID + 1) % THERMISTOR_COUNT;
}

void loop() {
  int current = millis();

  if (current >= nextTemp) {
    nextTemp += 1000 / FREQUENCY_HZ;
    readTemperatures();
    calculateStatistics();
    sendThermistorModule();
    sendThermistorGeneral();
    // Serial.printf("Temp (%d)\n", current);
  }

  if (current >= nextClaimAddress) {
    nextClaimAddress += 1000 / FREQUENCY_ADDRESS_CLAIM_HZ;
    sendAddressClaim();
    // Serial.printf("Claim address (%d)\n", current);
  }

  // Wait time to keep frequency
  current = millis();
  int timeToWait = min(nextTemp - current, nextClaimAddress - current);
  if (timeToWait > 0) {
    delay(timeToWait);
  }
}
