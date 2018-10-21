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

static const uint8_t sensor_pins[] = {ISENSE_RB, ISENSE_MOT, ISENSE_MAIN, ISENSE_PLD, ISENSE_SD, ISENSE_GPS, VSENSE_MAIN, VSENSE_CAP};
static const float sensor_gains[] = {1000/0.54,1000/0.54,1000/0.54,1000/0.54,1000/5.4,1000/5.4, 5.99, 5.99};
static const int num_sensors = sizeof(sensor_pins);
static const int sensor_repeat = 1;
volatile static int sensor_accum[num_sensors + 1] = {0};
volatile static int sensor_n = 0;

class Sensors {
public:

/**********************************  SETUP  ***********************************/
  Sensors() :
    bme1(BMP_CS_ONE),
    bme2(BMP_CS_TWO),
    bme3(BMP_CS_THREE),
    bme4(BMP_CS_FOUR),
    bmp1(BMPX_CS_TWO),
    bmp2(BMPX_CS_TWO) {
  }
  bool  init();

/********************************  FUNCTIONS  *********************************/
  float getVoltageSuperCap();
  float getJoules();
  float getDerivedTemp(uint8_t sensor);
  float getRawTemp(uint8_t sensor);
  float getRawPressure(uint8_t sensor);
	float getSensor(int sensor);
  float getTime();
	void reset();

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

	IntervalTimer sensorTimer;

	private:
};

#endif
