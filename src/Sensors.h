/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu
  Jesus Cervantes | cerjesus@stanford.edu

  File: Sensors.h
  --------------------------
  Interface to sensors on avionics hardware.
*/

#ifndef SENSORS_H
#define SENSORS_H

#include "Config.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <LTC2991.h>

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
  double   getVoltage();
  double   getCurrent();
  double   getJoules();
  double   getCurrentGPS();
  double   getCurrentRB();
  double   getCurrentMotors();
  double   getCurrentPayload();
  double   getNeckTemp();
  double   getRawTemp(uint8_t sensor);
  double   getRawPressure(uint8_t sensor);
  double   getRawAltitude(uint8_t sensor);
private:
/*********************************  HELPERS  **********************************/
/*********************************  OBJECTS  **********************************/
  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_BMP280 bme3;
  Adafruit_BMP280 bme4;

};

#endif
