/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.h
  --------------------------
  Interface to guarenteed good sensor values.
*/

#ifndef FILTERS_H
#define FILTERS_H

#include "Config.h"
#include <Eigen30.h>

class Filters {
public:

/**********************************  SETUP  ***********************************/
  bool     init();
/********************************  FUNCTIONS  *********************************/
  void     enableSensors(bool BMP1Enable, bool BMP2Enable, bool BMP3Enable, bool BMP4Enable);
  double   getTemp(double RAW_TEMP_1,double RAW_TEMP_2,double RAW_TEMP_3,double RAW_TEMP_4);
  double   getPressure(double RAW_PRESSURE_1,double RAW_PRESSURE_2,double RAW_PRESSURE_3,double RAW_PRESSURE_4);
  float    getCalculatedAltitude(float pressure, float pressureBaseline);
  double   getAltitude(double RAW_ALTITUDE_1,double RAW_ALTITUDE_2,double RAW_ALTITUDE_3,double RAW_ALTITUDE_4);
  double   getAscentRate();
private:
/*********************************  HELPERS  **********************************/
/*********************************  OBJECTS  **********************************/
  bool     enabledSensors[4] = {true};
  uint8_t  numSensors;
  float    ASCENT_RATE_BUFFER[BUFFER_SIZE];
  uint16_t ascentRateIndex = 0;
  double   altitudeCurr;
  double   altitudeLast;
  uint64_t ascentRateLast;

};

#endif
