/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu

  File: Charger.h
  --------------------------
  Interface to PCB Charger.
*/

#ifndef CHARGER_H
#define CHARGER_H

#include "Config.h"
#include <AD5246.h>

class Charger {
public:
/**********************************  SETUP  ***********************************/
  bool init();
/********************************  FUNCTIONS  *********************************/
  void enable5VBoost();
  void disable5VBoost();
  void runCharger(uint8_t hex);
private:
/*********************************  HELPERS  **********************************/

/*********************************  OBJECTS  **********************************/
  AD5246 resistor;
};

#endif
