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
float Controller::getValveIncentive(float valveAltitudeSetpoint, float valveKpConstant, float valveKdConstant, float valveKiConstant, double ascentRate, double altitude, double altitudeSinceLastVent) {
  float proportionalTerm = valveKpConstant * (altitude - valveAltitudeSetpoint);
  float derivitveTerm    = valveKdConstant * ascentRate;
  float integralTerm     = valveKiConstant * (altitude - altitudeSinceLastVent);
  return proportionalTerm + derivitveTerm + integralTerm;
}

/*
 * Function: getBalastIncentive
 * -------------------
 * This function calculates the current incentive to actuate the balast.
 */
float Controller::getBalastIncentive(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKdConstant, float ballastKiConstant, double ascentRate, double altitude, double altitudeSinceLastDrop) {
  float proportionalTerm = ballastKpConstant * (ballastAltitudeSetpoint - altitude);
  float derivitveTerm    = ballastKdConstant * -1 * ascentRate;
  float integralTerm     = ballastKiConstant * (altitudeSinceLastDrop - altitude);
  return proportionalTerm + derivitveTerm + integralTerm;
}
