/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu

  File: AD5246.cpp
  --------------------------
  Implimentation of AD5246.h
*/

#include "AD5246.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the AD5246 resistor.
 */
bool AD5246::init() {
  setResistance(0x10);
  delay(100);
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: setResistance
 * -------------------
 * This function sets the resistor to the specified hex value.
 */
bool AD5246::setResistance(uint8_t hex) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(byte(hex));
  Wire.endTransmission();
  return true;
}
