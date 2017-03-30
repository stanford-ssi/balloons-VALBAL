/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Ella Hofmann-Coyle | ellahofm@stanford.edu
  Claire Huang | chuang20@stanford.edu

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
 * Function: updateControllerConstants
 * -------------------
 * This function updates the constants to edit the algorythm.
 */
void Controller::updateControllerConstants(float incentiveThreshold, float reArmConstant, float BallastArmAlt) {
  INCENTIVE_THRESHOLD = incentiveThreshold;
  RE_ARM_CONSTANT     = reArmConstant;
  BALLAST_ARM_ALT     = BallastArmAlt;
}

/*
 * Function: updateValveConstants
 * -------------------
 * This function updates the constants to tune the algorythm.
 */
void Controller::updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant) {
  VALVE_SETPOINT               = valveAltitudeSetpoint;
  VALVE_VELOCITY_CONSTANT      = valveKpConstant;
  VALVE_ALTITUDE_DIFF_CONSTANT = valveKiConstant;
  VALVE_LAST_ACTION_CONSTANT   = valveKdConstant;
}

/*
 * Function: updateBallastConstants
 * -------------------
 * This function updates the constants to tune the algorythm.
 */
void Controller::updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant) {
  BALLAST_SETPOINT               = ballastAltitudeSetpoint;
  BALLAST_VELOCITY_CONSTANT      = ballastKpConstant;
  BALLAST_ALTITUDE_DIFF_CONSTANT = ballastKiConstant;
  BALLAST_LAST_ACTION_CONSTANT   = ballastKdConstant;
}

// RE_ARM_CONSTANT = INCENTIVE_THRESHOLD / (BALLAST_ALTITUDE_DIFF_CONSTANT + BALLAST_LAST_ACTION_CONSTANT);

/*
 * Function: getValveIncentive
 * -------------------
 * This function calculates the incentive to actuate the valve based on a PID
 * feedback controller.
 */
float Controller::getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVent) {
  // altitudeSinceLastVent = min(altitudeSinceLastVent, DAlt + reArmConst);
  float proportionalTerm = VALVE_VELOCITY_CONSTANT      * ascentRate;
  float integralTerm     = VALVE_ALTITUDE_DIFF_CONSTANT * (altitude - VALVE_SETPOINT);
  float derivativeTerm   = VALVE_LAST_ACTION_CONSTANT   * (altitude - altitudeSinceLastVent);
  return proportionalTerm + integralTerm + derivativeTerm;
}

/*
 * Function: getBallastIncentive
 * -------------------
 * This function calculates the incentive to actuate the ballast based on a PID
 * feedback controller.
 */
float Controller::getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDrop) {
  // altitudeSinceLastDrop = max(altitudeSinceLastDrop, DAlt- reArmConst);
  float proportionalTerm = BALLAST_VELOCITY_CONSTANT * -1 * ascentRate;
  float integralTerm     = BALLAST_ALTITUDE_DIFF_CONSTANT * (BALLAST_SETPOINT - altitude);
  float derivativeTerm   = BALLAST_LAST_ACTION_CONSTANT   * (altitudeSinceLastDrop - altitude);
  return proportionalTerm + integralTerm + derivativeTerm;
}
