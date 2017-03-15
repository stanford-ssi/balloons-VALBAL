/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.h
  --------------------------
  Interface to guarenteed good semsor values.
*/

#ifndef FILTERS_H
#define FILTERS_H

#include "Config.h"

class Filters {
public:

/**********************************  SETUP  ***********************************/
  Filters() :
    temp(0) {
  }

  bool     init();
/********************************  FUNCTIONS  *********************************/
  double   getTemp(double RAW_TEMP_1,double RAW_TEMP_2,double RAW_TEMP_3,double RAW_TEMP_4);
  double   getPressure(double RAW_PRESSURE_1,double RAW_PRESSURE_2,double RAW_PRESSURE_3,double RAW_PRESSURE_4);
  double   getAltitude(double RAW_ALTITUDE_1,double RAW_ALTITUDE_2,double RAW_ALTITUDE_3,double RAW_ALTITUDE_4);
  double   getAscentRate();
private:
/*********************************  HELPERS  **********************************/
/*********************************  OBJECTS  **********************************/
uint8_t temp;

  float    ASCENT_RATE_BUFFER[BUFFER_SIZE];
  uint16_t ascentRateIndex = 0;
  double   altitudeCurr;
  double   altitudeLast;
  uint64_t ascentRateLast;

};

#endif
