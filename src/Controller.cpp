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
float Controller::getValveIncentive(float valveKpConstant, float valveKdConstant, float valveKiConstant, double ascentRate) {
  float DAlt = 0;
  float ALTITUDE_SETPOINT = 0;
  float altitudeSinceLastVent = 0;
  return (valveKpConstant * ascentRate) +  (valveKdConstant * (DAlt - ALTITUDE_SETPOINT)) + (valveKiConstant * (DAlt - altitudeSinceLastVent));
}

/*
 * Function: getBalastIncentive
 * -------------------
 * This function calculates the current incentive to actuate the balast.
 */
float Controller::getBalastIncentive(float balastKpConstant, float balastKdConstant, float balastKiConstant, double ascentRate) {
  float DAlt = 0;
  float BALLAST_ALTITUDE_SETPOINT = 0;
  float altitudeSinceLastDrop = 0;
  return (-1 * balastKpConstant * ascentRate) + (balastKdConstant * (BALLAST_ALTITUDE_SETPOINT - DAlt)) + (balastKiConstant * (altitudeSinceLastDrop - DAlt));
}
