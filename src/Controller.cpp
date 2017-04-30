/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Controller.cpp
  --------------------------
  Implimentation of Controller.h
*/

#include "Controller.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the flight controller.
 */
bool Controller::init() {
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateValveConstants
 * -------------------
 * This function updates the constants to tune the algorithm.
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
 * This function updates the constants to tune the algorithm.
 */
void Controller::updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant) {
  BALLAST_SETPOINT               = ballastAltitudeSetpoint;
  BALLAST_VELOCITY_CONSTANT      = ballastKpConstant;
  BALLAST_ALTITUDE_DIFF_CONSTANT = ballastKiConstant;
  BALLAST_LAST_ACTION_CONSTANT   = ballastKdConstant;
}

/*
 * Function: updateControllerConstants
 * -------------------
 * This function updates the constants to edit the algorithm.
 */
float Controller::updateControllerConstants(float BallastArmAlt, float incentiveThreshold) {
  BALLAST_ARM_ALT = BallastArmAlt;
  RE_ARM_CONSTANT = incentiveThreshold / (BALLAST_ALTITUDE_DIFF_CONSTANT + BALLAST_LAST_ACTION_CONSTANT);
  return RE_ARM_CONSTANT;
}

/*
 * Function: getValveIncentive
 * -------------------
 * This function calculates the incentive to actuate the valve based on a PID
 * feedback controller.
 */
float Controller::getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVent) {
  float altitudeSinceLastVentCorrected = min(altitudeSinceLastVent, altitude + RE_ARM_CONSTANT);
  float proportionalTerm = VALVE_VELOCITY_CONSTANT      * ascentRate;
  float integralTerm     = VALVE_ALTITUDE_DIFF_CONSTANT * (altitude - VALVE_SETPOINT);
  float derivativeTerm   = VALVE_LAST_ACTION_CONSTANT   * (altitude - altitudeSinceLastVentCorrected);
  return proportionalTerm + integralTerm + derivativeTerm;
}

/*
 * Function: getBallastIncentive
 * -------------------
 * This function calculates the incentive to actuate the ballast based on a PID
 * feedback controller.
 */
float Controller::getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDrop) {
  float altitudeSinceLastDropCorrected = altitudeSinceLastDrop;
  if (!firstBallastDropped && altitude >= BALLAST_ARM_ALT && altitudeSinceLastDrop == BALLAST_ALT_LAST_DEFAULT) {
    altitudeSinceLastDropCorrected = BALLAST_ALT_LAST_FILLER;
    firstBallastDropped = true;
  }
  if(firstBallastDropped) altitudeSinceLastDropCorrected = max(altitudeSinceLastDropCorrected, altitude - RE_ARM_CONSTANT);
  float proportionalTerm = BALLAST_VELOCITY_CONSTANT * -1 * ascentRate;
  float integralTerm     = BALLAST_ALTITUDE_DIFF_CONSTANT * (BALLAST_SETPOINT - altitude);
  float derivativeTerm   = BALLAST_LAST_ACTION_CONSTANT   * (altitudeSinceLastDropCorrected - altitude);
  return proportionalTerm + integralTerm + derivativeTerm;
}

/*
 * Function: getIncentiveNoise
 * -------------------
 * This function calculates the inherent noise of the incentive.
 */
float Controller::getIncentiveNoise(bool IncludeBMP1, bool IncludeBMP2, bool IncludeBMP3, bool IncludeBMP4) {
  float incentiveNoise = 0;
  if(!IncludeBMP1) incentiveNoise++;
  if(!IncludeBMP2) incentiveNoise++;
  if(!IncludeBMP3) incentiveNoise++;
  if(!IncludeBMP4) incentiveNoise++;
  return incentiveNoise;
}
