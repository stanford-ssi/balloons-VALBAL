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
uint8_t temp;

};

#endif
