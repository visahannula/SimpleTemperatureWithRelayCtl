/***
 * Basic temperature monitoring and management with Arduino
 * 
 * This code reads temperature from a temperature sensor and
 * then control a relay (assumed a heater).
 * 
 * # Serial commands:
 * ## Get temperature
 *  - Command T
 *  - Returns all the connected sensors temperatures
 * 
 * ## Set temperature
 *  - Command S<sensor bus num>[H|L] <float temp>
 *  - Allows to set high temp with S (ex. S1H 22.1)
 *  - Set low temp (ex. S1L 20.5)
 * 
 * 
 * This is made with Arduino Uno board, DS18B20 Temperature sensors and
 * a relay board.
 * 
 * 2020 Visa Hannula (visa at viha.fi).
 * 
 * USE AT YOUR OWN RISK, NO WARRANTY.
 * 
 */
#include <Arduino.h>
#include <stdbool.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define SER_BAUDRATE 57600
#define ONE_WIRE_BUS 2
#define PORT_RELAY1 4
#define MAX_SENSORS 2
#define TEMP_DEF_HIGH 22.0
#define TEMP_DEF_LOW 21.0

// TODO: Handle on the fly removed and added sensors gracefully

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

struct Relay
{
  uint8_t state;
  unsigned int port;
  unsigned long lastSet;
};

struct Led
{
  unsigned int port;
  unsigned int state;
  unsigned int interval; // blinking speed
  unsigned long lastLEDMillis;
};

struct TempSensor
{
  float curr_temp;
  float prev_temp;
  float high_temp;
  float low_temp;
  bool isHighTemp; // TODO: use enum for these and check bit value
  bool isLowTemp;
  unsigned long lastUpdate;
  uint8_t deviceAddr[8];
};

TempSensor tempSensors[MAX_SENSORS];

uint8_t incomingByte;
uint8_t buffer[64];
uint8_t sensorNum;
uint8_t readBytes = 0; // count bytes read from serial, Uno buffer is 64 bytes as default

const char cmdCharSet = 'S';
const char cmdCharGetTemp = 'T';

const unsigned int tempInterval = 5000; // interval for sampling temp

struct Relay relay1;
struct Led internalLED;

uint8_t sensorCount = 0;

// SETUP
void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PORT_RELAY1, OUTPUT);

  internalLED.port = LED_BUILTIN;
  internalLED.interval = 1000;
  internalLED.lastLEDMillis = millis();
  setStateLED(&internalLED, HIGH);

  relay1.port = PORT_RELAY1;
  setStateRelay(&relay1, LOW);

  sensors.begin();
  delay(200);
  sensorCount = sensors.getDeviceCount();
  if (sensorCount > MAX_SENSORS)
    sensorCount = MAX_SENSORS;

  // initialize sensors
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    tempSensors[i] = {
        .curr_temp = -127.0, //initialize with fault indication
        .prev_temp = -127.0,
        .high_temp = TEMP_DEF_HIGH,
        .low_temp = TEMP_DEF_LOW,
        .isHighTemp = false,
        .isLowTemp = false,
        .lastUpdate = millis(),
        .deviceAddr = {0, 0, 0, 0, 0, 0, 0, 0}};

    sensors.getAddress(tempSensors[i].deviceAddr, i); // store addr to struct
    delay(200);
  }

  Serial.setTimeout(1000);
  Serial.begin(SER_BAUDRATE);

  if (Serial)
  {
    Serial.println(F("Temperature control v0.1b"));
    Serial.print(F("Number of devices:"));
    Serial.println(sensorCount);
    Serial.flush();

    printSensors(sensorCount);
  }
}

void printSensors(uint8_t sensor_cnt)
{
  Serial.write('[');
  for (uint8_t i = 0; i < sensor_cnt; i++)
  {
    Serial.print(F("{\"device_bus_num\":"));
    Serial.print(i);
    Serial.print(F(",\"device_addr_u8x\":"));
    Serial.write('"');
    printDevAddressSer(tempSensors[i].deviceAddr);
    Serial.print("\"}");
    if (sensorCount > 1 && i < sensorCount - 1)
    {
      Serial.write(',');
    }
    Serial.flush();
  }
  Serial.write(']');
}

/**
 *  Print a device address to serial as hex with ":" as separator
 */
void printDevAddressSer(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < HEX)
      Serial.write('0');
    Serial.print(deviceAddress[i], HEX);
    if (i < 7)
      Serial.write(':');
  }
}

void outputTempAll(TempSensor tempSensors[], uint8_t sensorCount)
{
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    outputTemp(tempSensors[i]);
  }
}

/** 
 * Serial output TempSensor in a JSON format.
 * Temp of -127 indicates a failure.
 */
void outputTemp(TempSensor sensor)
{
  Serial.print('{');
  Serial.print(F("\"temperature\":"));
  Serial.print(sensor.curr_temp);
  Serial.print(",");

  Serial.print(F("\"temperature_prev\":"));
  Serial.print(sensor.prev_temp);
  Serial.print(",");

  Serial.print(F("\"tempHighValue\":"));
  Serial.print(sensor.high_temp);
  Serial.print(",");

  Serial.print(F("\"tempLowValue\":"));
  Serial.print(sensor.low_temp);
  Serial.print(",");

  Serial.print(F("\"tempIsHigh\":"));
  Serial.print(sensor.isHighTemp ? F("true") : F("false"));
  Serial.print(",");

  Serial.print(F("\"tempIsLow\":"));
  Serial.print(sensor.isLowTemp ? F("true") : F("false"));
  Serial.print(",");

  Serial.print(F("\"unit\":\"Celsius\""));

  Serial.print('}');
  Serial.println();
  Serial.flush();
}

/**
 * Set high value for sensor
 * 
 * Checks that value is not same or lower than low.
 * 
 * Output sensor state to serial.
 */
void setHighTemp(TempSensor *sensor, float value)
{
  if (value > sensor->low_temp)
  {
    sensor->high_temp = value;
  }
  outputTemp(*sensor);
}

/**
 * Set low value for sensor
 * 
 * Checks that value is not same or higher than high.
 * 
 * Output sensor state to serial.
 */
void setLowTemp(TempSensor *sensor, float value)
{
  if (value < sensor->high_temp)
  {
    sensor->low_temp = value;
  }
  return outputTemp(*sensor);
}

void setStateLED(Led *led, int state)
{
  digitalWrite(led->port, state);
  led->state = state;
}

void setStateRelay(Relay *r, int state)
{
  digitalWrite(r->port, state);
  r->state = state;
  r->lastSet = millis();
}

/***
 * Updates temp value for the sensor
 */
void updateTempValue(TempSensor *sensor, float value)
{
  sensor->prev_temp = sensor->curr_temp;
  sensor->curr_temp = value;

  if (sensor->curr_temp >= sensor->high_temp)
  {
    sensor->isHighTemp = true;
    sensor->isLowTemp = false;
  }
  else if (sensor->curr_temp <= sensor->low_temp)
  {
    sensor->isHighTemp = false;
    sensor->isLowTemp = true;
  }
  else
  {
    sensor->isHighTemp = false;
    sensor->isLowTemp = false;
  }

  sensor->lastUpdate = millis();
}

/***
 * Empty buffer by reading all the bytes
 */
void emptySerialRecvBuf(void)
{
  while (Serial.available() > 0)
  {
    Serial.read(); // empty recv buffer
  }
}

uint8_t readSerialBuffer(uint8_t *buffer, size_t bufSize, uint8_t *readBytes)
{
  *readBytes = 0;

  if (Serial.available() > 0)
  {
    *readBytes = Serial.readBytesUntil('\n', buffer, bufSize - 1);
    emptySerialRecvBuf(); // don't want to process more commands
  }

  buffer[*readBytes] = '\0'; // NULL terminate the string

  if (readBytes > 0)
  {
    // print status
    Serial.print(F("Read "));
    Serial.print(*readBytes);
    Serial.print(F(" bytes: "));
    Serial.println((char *)buffer);
    Serial.flush();
  }

  return *readBytes;
}

void processBufferCommand(uint8_t *buffer, int readBytes)
{
  switch (buffer[0])
  {
  case cmdCharGetTemp:                       // TEMP request
    outputTempAll(tempSensors, sensorCount); // TODO: Handle individual sensors
    break;

  case cmdCharSet:
    processSetCommand(buffer, readBytes);
    break;

  default:
    break;
  }
}

// SET request
// Format: S<sensor digit><temp><newline> ("S1H 22.2\n")
void processSetCommand(uint8_t *buffer, int readBytes)
{
  if (readBytes < 4)
    return; // need at least single number to set

  sensorNum = buffer[1] - '0'; // ascii number conversion

  if (sensorNum < 0 && sensorNum >= MAX_SENSORS)
    return;

  Serial.print(F("Set temp for sensor: "));
  Serial.print(sensorNum);
  Serial.flush();

  Serial.print(F(" Buffer:\""));
  for (int i = 0; i <= readBytes; i++)
  {
    Serial.write(buffer[i]);
  }
  Serial.println('"');

  if (buffer[2] == 'H') // High Temp
  {
    Serial.println(F("Operation to set high temp."));
    setHighTemp(&tempSensors[sensorNum], (float)strtod((char *)buffer + 3, NULL));
  }
  else if (buffer[2] == 'L') // Low temp
  {
    Serial.println(F("Operation to set low temp."));
    setLowTemp(&tempSensors[sensorNum], (float)strtod((char *)buffer + 3, NULL));
  }
  else
  {
    Serial.println(F("ERROR: Syntax error."));
  }
  Serial.flush();
}

/**
 * Actual program loop
 * 
 */
void loop(void)
{
  // temperature measurement
  if (tempSensors[0].lastUpdate < (millis() - tempInterval))
  {
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    sensors.requestTemperatures(); // Send the command to get temperature readings

    // You can have more than one DS18B20 on the same bus.
    // 0 refers to the first IC on the wire
    //updateTempValue(&tempSensors[0], sensors.getTempCByIndex(0));
    updateTempValue(&tempSensors[0], sensors.getTempC(tempSensors[0].deviceAddr));
  }

  if (readSerialBuffer(buffer, sizeof(buffer), &readBytes))
  {
    setStateLED(&internalLED, HIGH);
    processBufferCommand(buffer, readBytes);
  };

  /**
   * Relay and LED control based on temp values

   If the temp is -127 we have a fault and set relay operating
   to prevent freezing. Connected heating equipment MUST have
   their own thermostat!
  */
  if (tempSensors[0].curr_temp <= -127.0)
  { // we have sensor fault
    internalLED.interval = 150;
    setStateRelay(&relay1, HIGH); // Set temp to high (to prevent freezing)
  }
  else if (tempSensors[0].isHighTemp)
  { // High temp reached
    internalLED.interval = 1000;
    setStateRelay(&relay1, LOW);
  }
  else if (tempSensors[0].isLowTemp)
  { // Low temp reached
    internalLED.interval = 250;
    setStateRelay(&relay1, HIGH);
  }
  else
  { // Hysteris region
    internalLED.interval = 2000;
  }

  // led blinker (internal led)
  if (internalLED.lastLEDMillis < (millis() - internalLED.interval))
  {
    setStateLED(&internalLED, internalLED.state == HIGH ? LOW : HIGH);
    internalLED.lastLEDMillis = millis();
  }
}
