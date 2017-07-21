/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
  Vaiva Snapkauskaite | vaivas@stanford.edu
  Davy Ragland        | dragland@stanford.edu

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
  bool success = false;
  return success;
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
