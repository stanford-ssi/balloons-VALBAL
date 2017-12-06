/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: ControllerLegacy.cpp
  --------------------------
  Implimentation of ControllerLegacy.h
*/

#include "ControllerLegacy.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the flight controller.
 */
bool ControllerLegacy::init() {
  return true;
}

/***************************** PUBLIC FUNCTIONS  ******************************/
/*
 * Function: updateConstants
 * -------------------
 * This function updates the valve, ballast, and controller constants to tune the algorithm.
 * (All Controllers Should Implement This Function)
 */
void ControllerLegacy::updateConstants(ControllerLegacyConstants constants) {
  updateValveConstants(constants.valveAltitudeSetpoint, constants.valveKpConstant, constants.valveKiConstant, constants.valveKdConstant);
  updateBallastConstants(constants.ballastAltitudeSetpoint, constants.ballastKpConstant, constants.ballastKiConstant, constants.ballastKdConstant);
  STATE.reArmConstant = updateControllerConstants(constants.BallastArmAlt, constants.incentiveThreshold);
}

/*
 * Function: update
 * -------------------
 * This function updates the inputs to the getValveIncentive and getBallastIncentive functions and
 * stores them in the controller's state.
 * (All Controllers Should Implement This Function)
 */
void ControllerLegacy::update(ControllerLegacyInputs inputs) {
  STATE.altitudeSinceLastVentCorrected = getAltitudeSinceLastVentCorrected(inputs.altitude, inputs.altitudeSinceLastVent);
  STATE.altitudeSinceLastDropCorrected = getAltitudeSinceLastDropCorrected(inputs.altitude, inputs.altitudeSinceLastDrop);
  STATE.altitude = inputs.altitude;
  STATE.ascentRate = inputs.ascentRate;
  STATE.valveIncentive = getValveIncentive(STATE.ascentRate, STATE.altitude, STATE.altitudeSinceLastVentCorrected);
  STATE.ballastIncentive = getBallastIncentive(STATE.ascentRate, STATE.altitude, STATE.altitudeSinceLastDropCorrected);
}

/*
 * Function: getAction
 * -------------------
 * This function returns a negative number for valve and a positive number for ballast
 * (All Controllers Should Implement This Function)
 */
float ControllerLegacy::getAction() {
  if (STATE.valveIncentive > STATE.ballastIncentive) {
    return -STATE.valveIncentive;
  } else {
    return STATE.ballastIncentive;
  }
}

ControllerLegacyState ControllerLegacy::getState() {
  return STATE;
}
/***************************** PRIVATE FUNCTIONS  *****************************/
/*
 * Function: updateValveConstants
 * -------------------
 * This function updates the constants to tune the algorithm.
 */
void ControllerLegacy::updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant) {
  CONSTANTS.valveAltitudeSetpoint    = valveAltitudeSetpoint;
  CONSTANTS.valveKpConstant          = valveKpConstant;
  CONSTANTS.valveKiConstant          = valveKiConstant;
  CONSTANTS.valveKdConstant          = valveKdConstant;
}

/*
 * Function: updateBallastConstants
 * -------------------
 * This function updates the constants to tune the algorithm.
 */
void ControllerLegacy::updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant) {
  CONSTANTS.valveAltitudeSetpoint   = ballastAltitudeSetpoint;
  CONSTANTS.ballastKpConstant       = ballastKpConstant;
  CONSTANTS.ballastKiConstant       = ballastKiConstant;
  CONSTANTS.ballastKdConstant       = ballastKdConstant;
}

/*
 * Function: updateControllerConstants
 * -------------------
 * This function updates the constants to edit the algorithm.
 */
float ControllerLegacy::updateControllerConstants(float BallastArmAlt, float incentiveThreshold) {
  CONSTANTS.BallastArmAlt = BallastArmAlt;
  return incentiveThreshold / (CONSTANTS.ballastKiConstant + CONSTANTS.ballastKdConstant);
}

/*
 * Function: getAltitudeSinceLastVentCorrected
 * -------------------
 * This function returns a corrected altitude since last vent value.
 */
float ControllerLegacy::getAltitudeSinceLastVentCorrected(double altitude, double altitudeSinceLastVent) {
  float altitudeSinceLastVentCorrected = min(altitudeSinceLastVent, altitude + STATE.reArmConstant);
  return altitudeSinceLastVentCorrected;
}

/*
 * Function: getAltitudeSinceLastDropCorrected
 * -------------------
 * This function returns a corrected altitude since last drop value.
 */
float ControllerLegacy::getAltitudeSinceLastDropCorrected(double altitude, double altitudeSinceLastDrop) {
  float altitudeSinceLastDropCorrected = altitudeSinceLastDrop;
  if (!firstBallastDropped && altitude >= CONSTANTS.BallastArmAlt && altitudeSinceLastDrop == BALLAST_ALT_LAST_DEFAULT) {
    altitudeSinceLastDropCorrected = BALLAST_ALT_LAST_FILLER;
    firstBallastDropped = true;
  }
  if(firstBallastDropped) altitudeSinceLastDropCorrected = max(altitudeSinceLastDropCorrected, altitude - STATE.reArmConstant);
  return altitudeSinceLastDropCorrected;
}

/*
 * Function: getValveIncentive
 * -------------------
 * This function calculates the incentive to actuate the valve based on a PID
 * feedback controller.
 */
float ControllerLegacy::getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVentCorrected) {
  float proportionalTerm = CONSTANTS.valveKpConstant   * ascentRate;
  float integralTerm     = CONSTANTS.valveKiConstant   * (altitude - CONSTANTS.valveAltitudeSetpoint);
  float derivativeTerm   = CONSTANTS.valveKdConstant   * (altitude - altitudeSinceLastVentCorrected);
  return proportionalTerm + integralTerm + derivativeTerm;
}

/*
 * Function: getBallastIncentive
 * -------------------
 * This function calculates the incentive to actuate the ballast based on a PID
 * feedback controller.
 */
float ControllerLegacy::getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDropCorrected) {
  float proportionalTerm = CONSTANTS.ballastKpConstant  * -1 * ascentRate;
  float integralTerm     = CONSTANTS.ballastKiConstant  * (CONSTANTS.ballastAltitudeSetpoint - altitude);
  float derivativeTerm   = CONSTANTS.ballastKdConstant  * (altitudeSinceLastDropCorrected - altitude);
  return proportionalTerm + integralTerm + derivativeTerm;
}
