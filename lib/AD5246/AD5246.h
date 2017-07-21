/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
  Vaiva Snapkauskaite | vaivas@stanford.edu
  Davy Ragland        | dragland@stanford.edu

  File: AD5246.h
  --------------------------
  Interface to 100kOhm i2c AD5246 resistor
*/

#ifndef AD5246_H
#define AD5246_H

#include <Arduino.h>

class AD5246 {
public:
/**********************************  SETUP  ***********************************/
  bool    init();
/********************************  FUNCTIONS  *********************************/
  bool    setResistance(float ohms);
  float   getCurrentResistance();
private:
/*********************************  HELPERS  **********************************/
  uint8_t ohmsToSteps(float ohms);
/*********************************  OBJECTS  **********************************/
  uint16_t ADDRESS;
  float    resistance;
};

#endif
