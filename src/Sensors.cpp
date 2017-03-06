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
  if (!bme1.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  if (!bme2.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  if (!bme3.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  if (!bme4.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  WireNew.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  WireNew.setDefaultTimeout(5 * 1000);
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
  This function returns the time as a fixed length string.
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
  return (double)analogRead(VBAT_PIN) * 1.2 * 4.0 / (double)pow(2, 12);
}

/*
  function: getCurrent
  ---------------------------------
  This function gets the total current draw.
*/
double Sensors::getCurrent() {
  double currentMonitor = (double)analogRead(BATT_CURRENT) / (double)pow(2, 12) * 1.2 * 4.0 / 0.496;
  double externalCurrentMonitor = (double)analogRead(EXTERNAL_CURRENT) / (double)pow(2, 12) * 1.2 * 4.0 / 0.496;
  return currentMonitor + externalCurrentMonitor;
}

/*
  function: getCurrentGPS
  ---------------------------------
  This function gets the GPS current draw.
*/
double Sensors::getCurrentGPS() {
  return 0;
}

/*
  function: getCurrentRB
  ---------------------------------
  This function gets the RockBLOCK current draw.
*/
double Sensors::getCurrentRB() {
  return 0;
}

/*
  function: getCurrentMotors
  ---------------------------------
  This function gets the motor current draw.
*/
double Sensors::getCurrentMotors() {
  return 0;
}

/*
  function: getCurrentPayload
  ---------------------------------
  This function gets the payload current draw.
*/
double Sensors::getCurrentPayload() {
  return 0;
}

/*
  function: getRawTemp
  ---------------------------------
  This function returns a raw reading from each of the sensors
*/
void Sensors::getRawTemp(double &RAW_TEMP_1,
                            double &RAW_TEMP_2,
                            double &RAW_TEMP_3,
                            double &RAW_TEMP_4) {

  RAW_TEMP_1 = bme1.readTemperature();
  RAW_TEMP_2 = bme2.readTemperature();
  RAW_TEMP_3 = bme3.readTemperature();
  RAW_TEMP_4 = bme4.readTemperature();
}

/*
  function: getRawPressure
  ---------------------------------
  This function returns a raw reading from each of the sensors
*/
void Sensors::getRawPressure(double &RAW_PRESSURE_1,
                              double &RAW_PRESSURE_2,
                              double &RAW_PRESSURE_3,
                              double &RAW_PRESSURE_4) {
  
  RAW_PRESSURE_1 = bme1.readPressure();
  RAW_PRESSURE_2 = bme2.readPressure();
  RAW_PRESSURE_3 = bme3.readPressure();
  RAW_PRESSURE_4 = bme4.readPressure();

}

/*
  function: getRawAltitude
  ---------------------------------
  This function returns a raw reading from each of the sensors
*/
void Sensors::getRawAltitude(double &RAW_ALTITUDE_1,
                              double &RAW_ALTITUDE_2,
                              double &RAW_ALTITUDE_3,
                              double &RAW_ALTITUDE_4) {

  RAW_ALTITUDE_1 = bme1.readAltitude(1013.25);
  RAW_ALTITUDE_2 = bme2.readAltitude(1013.25);
  RAW_ALTITUDE_3 = bme3.readAltitude(1013.25);
  RAW_ALTITUDE_4 = bme4.readAltitude(1013.25);

}


