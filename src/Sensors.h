/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
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
#include <DFRobot_BMP388.h>
#include <LTC2991.h>

class Sensors {
public:

/**********************************  SETUP  ***********************************/
  Sensors() :
    bme1(BMP_CS_ONE),
    bme2(BMP_CS_TWO),
    bme3(BMP_CS_THREE),
    bme4(BMP_CS_FOUR),
    bmp1(BMPX_CS_ONE),
    bmp2(BMPX_CS_ONE) {
  }
  bool  init();

/********************************  FUNCTIONS  *********************************/
  float getVoltagePrimary();
  float getVoltageSuperCap();
  float getCurrentTotal();
  float getCurrentSubsystem(uint8_t subsystem);
  float getJoules();
  float getDerivedTemp(uint8_t sensor);
  float getRawTemp(uint8_t sensor);
  float getRawPressure(uint8_t sensor);

/*********************************  OBJECTS  **********************************/
  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_BMP280 bme3;
  Adafruit_BMP280 bme4;
	DFRobot_BMP388 bmp1;
	DFRobot_BMP388 bmp2;
  uint32_t lastJoulesCall = 0;
  float internalCurrentMonitor = 0;
  float voltageSuperCap = 0;
  float voltagePrimary = 0;
  float joules = 0;
	private:
};

#endif
