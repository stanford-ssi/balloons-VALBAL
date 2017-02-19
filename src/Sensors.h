/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Sensors.h
  --------------------------
  Interface to sensors on avionics hardware.
*/

#ifndef SENSORS_H
#define SENSORS_H

#include "Config.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>
// #include <LTC2991.h>

class Sensors {
public:

/**********************************  SETUP  ***********************************/
  Sensors() :
    bme1(BMP_CS_ONE),
    bme2(BMP_CS_TWO),
    bme3(BMP_CS_THREE),
    bme4(BMP_CS_FOUR) {
  }
  bool     init();
/********************************  FUNCTIONS  *********************************/
  uint32_t getTime();
  double   getVoltage();
  double   getCurrent();
  double   getCurrentGPS();
  double   getCurrentRB();
  double   getCurrentMotors();
  double   getCurrentPayload();
  double   getTemp();
  double   getPressure();
  double   getAltitude();
  double   getAscentRate();
private:
/*********************************  HELPERS  **********************************/
/*********************************  OBJECTS  **********************************/
  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_BMP280 bme3;
  Adafruit_BMP280 bme4;
  float    ASCENT_BUFFER[BUFFER_SIZE];
  double   ALTITUDE_CURR;
  double   ALTITUDE_LAST;
  uint64_t ASCENT_RATE_LAST;

};

#endif
