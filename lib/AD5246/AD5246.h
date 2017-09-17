/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu

  File: AD5246.h
  --------------------------
  Interface to 100kOhm i2c AD5246 resistor.
*/

#ifndef AD5246_H
#define AD5246_H

#include <Arduino.h>
#include <LTC2991.h>

class AD5246 {
public:
/**********************************  SETUP  ***********************************/
  bool init();
/********************************  FUNCTIONS  *********************************/
  bool setResistance(uint8_t hex);
private:
/*********************************  OBJECTS  **********************************/
  uint16_t ADDRESS = 0x2E;
};

#endif
