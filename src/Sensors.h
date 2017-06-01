/*
  Stanford Student Space Initiative
  Balloons | VALBAL | June 2017
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
  float    getVoltagePrimary();
  float    getVoltage5V();
  float    getCurrent();
  float    getCurrentSubsystem(uint8_t subsystem);
  float    getJoules();
  float    getDerivedTemp(uint8_t sensor);
  float    getRawTemp(uint8_t sensor);
  float    getRawPressure(uint8_t sensor);
private:
/*********************************  OBJECTS  **********************************/
  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_BMP280 bme3;
  Adafruit_BMP280 bme4;
  uint32_t lastJoulesCall = 0;
  float internalCurrentMonitor = 0;
  float externalCurrentMonitor = 0;
  float voltagePrimary = 0;
  float voltage5V = 0;
  float joules = 0;
};

#endif
