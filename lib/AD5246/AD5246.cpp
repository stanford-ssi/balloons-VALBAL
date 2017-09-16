/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu

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
  wire.beginTransmission(0x2E);
  wire.write(byte(0x10));
  wire.endTransmission();
  delay(100);
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: setResistance
 * -------------------
 * This function sets the resistor to the desired resistance.
 */
bool AD5246::setResistance(float ohms) {
  bool success = false;
  uint8_t step = ohmsToSteps(ohms);
  return success;
}

/*
 * Function: getCurrentResistance
 * -------------------
 * This function gets the current resistance that has been stepped to.
 */
float AD5246::getCurrentResistance() {
  return resistance;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: ohmsToSteps
 * -------------------
 * This function converts an arbritary resistance to the discrite
 * step needed for the 128 step resistor.
 */
uint8_t AD5246::ohmsToSteps(float ohms) {
  return 0;
}


// void highBoostPower(){
//     Wire.beginTransmission(0x2E);
//     Wire.write(byte(0x10));
//     Wire.endTransmission();
// }
//
// /*************************************************************************************************************************************************************************************************/
//
// void lowBoostPower(){
//     Wire.beginTransmission(0x2E);
//     Wire.write(byte(0x7F));
//     Wire.endTransmission();
// }
