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
 * This function calculates the incentive to actuate the valve based on a PID
 * feedback controller.
 */
float Controller::getValveIncentive(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant, double ascentRate, double altitude, double altitudeSinceLastVent) {
  float proportionalTerm = valveKpConstant * ascentRate;
  float integralTerm     = valveKiConstant * (altitude - valveAltitudeSetpoint);
  float derivitveTerm    = valveKdConstant * (altitude - altitudeSinceLastVent);
  return proportionalTerm + integralTerm + derivitveTerm;
}

/*
 * Function: getBalastIncentive
 * -------------------
 * This function calculates the incentive to actuate the balast based on a PID
 * feedback controller.
 */
float Controller::getBalastIncentive(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant, double ascentRate, double altitude, double altitudeSinceLastDrop) {
  float proportionalTerm = ballastKpConstant * -1 * ascentRate;
  float integralTerm     = ballastKiConstant * (ballastAltitudeSetpoint - altitude);
  float derivitveTerm    = ballastKdConstant * (altitudeSinceLastDrop - altitude);
  return proportionalTerm + integralTerm + derivitveTerm;
}
