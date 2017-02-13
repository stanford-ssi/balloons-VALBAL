/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Ella Hofmann-Coyle | ellahofm@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Controller.cpp
  --------------------------
  Implimentation of Controller.h
*/

#include "Controller.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the flight controller.
*/
bool Controller::init() {
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: getValveIncentive
 * -------------------
 * This function calculates the current incentive to actuate the valve.
 */
double Controller::getValveIncentive() {
  return 0;
}

/*
 * Function: getBalastIncentive
 * -------------------
 * This function calculates the current incentive to actuate the balast.
 */
double Controller::getBalastIncentive() {
  return 0;
}
