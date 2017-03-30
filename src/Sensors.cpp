/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Sensors.cpp
  --------------------------
  Implimentation of Sensors.h
*/

#include "Sensors.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the sensor hardware.
*/
bool Sensors::init() {
  bool sucess = true;
  pinMode(BATT_VOLTAGE,     INPUT);
  pinMode(BATT_CURRENT,     INPUT);
  pinMode(EXTERNAL_CURRENT, INPUT);
  pinMode(NECK_TEMP_SENSOR, INPUT);
  if (!bme1.begin()) {
    Serial.println("Could not initialize BMP280 sensor 1, check wiring!");
    sucess = false;
  }
  if (!bme2.begin()) {
    Serial.println("Could not initialize BMP280 sensor 2, check wiring!");
    sucess = false;
  }
  if (!bme3.begin()) {
    Serial.println("Could not initialize BMP280 sensor 3, check wiring!");
    sucess = false;
  }
  if (!bme4.begin()) {
    Serial.println("Could not initialize BMP280 sensor 4, check wiring!");
    sucess = false;
  }
  Serial.println(bme1.readPressure());
  Serial.println(bme2.readPressure());
  Serial.println(bme3.readPressure());
  Serial.println(bme4.readPressure());
  WireNew.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  WireNew.setDefaultTimeout(5000);
  int8_t ack = 0;
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CHANNEL_ENABLE_REG, LTC2991_ENABLE_ALL_CHANNELS);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_V1234_REG, 0x00);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_V5678_REG, 0x00);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_PWM_Tinternal_REG, LTC2991_REPEAT_MODE);
  return sucess;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: getTime
  ---------------------------------
  This function returns the time.
*/
uint32_t Sensors::getTime() {
  return millis();
}

/*
  function: getVoltage
  ---------------------------------
  This function gets the battery voltage.
*/
double Sensors::getVoltage() {
  return (double)analogRead(BATT_VOLTAGE) * 1.2 * 4.0 / (double)pow(2, 12);
}

/*
  function: getCurrent
  ---------------------------------
  This function gets the total current draw.
*/
double Sensors::getCurrent() {
  double internalCurrentMonitor = (double)analogRead(BATT_CURRENT)     / (double)pow(2, 12) * 1.2 * 4.0 / 0.496;
  double externalCurrentMonitor = (double)analogRead(EXTERNAL_CURRENT) / (double)pow(2, 12) * 1.2 * 4.0 / 0.496;
  return internalCurrentMonitor + externalCurrentMonitor;
}

/*
  function: getCurrentGPS
  ---------------------------------
  This function gets the GPS current draw.
*/
double Sensors::getCurrentGPS() {
  float currentMonitor = readCurrent(1);
  return currentMonitor;
}

/*
  function: getCurrentRB
  ---------------------------------
  This function gets the RockBLOCK current draw.
*/
double Sensors::getCurrentRB() {
  float currentMonitor = readCurrent(2);
  return currentMonitor;
}

/*
  function: getCurrentMotors
  ---------------------------------
  This function gets the motor current draw.
*/
double Sensors::getCurrentMotors() {
  float currentMonitor = readCurrent(3);
  return currentMonitor;
}

/*
  function: getCurrentPayload
  ---------------------------------
  This function gets the payload current draw.
*/
double Sensors::getCurrentPayload() {
  float currentMonitor = readCurrent(4);
  return currentMonitor;
}

/*
  function: getNeckTemp
  ---------------------------------
  This function gets the balloon neck temperature.
*/
double Sensors::getNeckTemp() {
  double vA = analogRead(NECK_TEMP_SENSOR) * 1.2 / (pow(2.0, 12.0));
  double x = log(vA * 100000.0 / (1.0 - vA));
  double a =   4.00141132e+02;
  double b =  -9.94189235e+01;
  double c =   1.16421122e+01;
  double d =  -9.42945754e-01;
  double e =   4.40544424e-02;
  double f =  -8.79058885e-04;
  return a + b * x + c * pow(x, 2) + d * pow(x, 3) + e * pow(x, 4) + f * pow(x, 5);
}

/*
  function: getRawTemp
  ---------------------------------
  This function returns a raw reading from each of the sensors.
*/
double Sensors::getRawTemp(uint8_t sensor) {
  double value =  -1;
  if (sensor == 1) value = bme1.readTemperature();
  if (sensor == 2) value = bme2.readTemperature();
  if (sensor == 3) value = bme3.readTemperature();
  if (sensor == 4) value = bme4.readTemperature();
  return value;
}

/*
  function: getRawPressure
  ---------------------------------
  This function returns a raw reading from each of the sensors.
*/
double Sensors::getRawPressure(uint8_t sensor) {
  double value =  -1;
  if (sensor == 1) value = bme1.readPressure();
  if (sensor == 2) value = bme2.readPressure();
  if (sensor == 3) value = bme3.readPressure();
  if (sensor == 4) value = bme4.readPressure();
  return value;
}

/*
  function: getRawAltitude
  ---------------------------------
  This function returns a raw reading from each of the sensors.
*/
double Sensors::getRawAltitude(uint8_t sensor) {
  double value =  -1;
  if (sensor == 1) value = bme1.readAltitude();
  if (sensor == 2) value = bme2.readAltitude();
  if (sensor == 3) value = bme3.readAltitude();
  if (sensor == 4) value = bme4.readAltitude();
  return value;
}
