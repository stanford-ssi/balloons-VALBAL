/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
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
  // upadate all the valve constants for all the controllers
  legacyController.updateValveConstants(valveAltitudeSetpoint, valveKpConstant,valveKiConstant, valveKdConstant);

}

/*
 * Function: updateBallastConstants
 * -------------------
 * This function updates the constants to tune the algorithm.
 */
void Controller::updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant) {
  // update all the ballast constants for all the controllers
  legacyController.updateBallastConstants(ballastAltitudeSetpoint, ballastKpConstant, ballastKiConstant, ballastKdConstant);

}

/*
 * Function: updateControllerConstants
 * -------------------
 * This function updates the constants to edit the algorithm.
 */
float Controller::updateControllerConstants(uint8_t controllerIndex, float BallastArmAlt, float incentiveThreshold) {
  float result = 0.0;
  if (controllerIndex == LEGACY_CONTROLLER_INDEX) {
    result = legacyController.updateControllerConstants(BallastArmAlt, incentiveThreshold);
  }
  return result;
}

/*
 * Function: getAltitudeSinceLastVentCorrected
 * -------------------
 * This function returns a corrected altitude since last vent value.
 */
float Controller::getAltitudeSinceLastVentCorrected(uint8_t controllerIndex, double altitude, double altitudeSinceLastVent) {
  float altitudeSinceLastVentCorrected = altitudeSinceLastVent;
  if (controllerIndex == LEGACY_CONTROLLER_INDEX) {
    altitudeSinceLastVentCorrected = legacyController.getAltitudeSinceLastVentCorrected(altitude, altitudeSinceLastVent);
  }
  return altitudeSinceLastVentCorrected;
}

/*
 * Function: getAltitudeSinceLastDropCorrected
 * -------------------
 * This function returns a corrected altitude since last drop value.
 */
float Controller::getAltitudeSinceLastDropCorrected(uint8_t controllerIndex, double altitude, double altitudeSinceLastDrop) {
  float altitudeSinceLastDropCorrected = altitudeSinceLastDrop;
  if (controllerIndex == LEGACY_CONTROLLER_INDEX) {
    altitudeSinceLastDropCorrected = legacyController.getAltitudeSinceLastDropCorrected(altitude, altitudeSinceLastDrop);
  }
  return altitudeSinceLastDropCorrected;
}

/*
 * Function: getValveIncentive
 * -------------------
 * This function calculates the incentive to actuate the valve based on a PID
 * feedback controller.
 */
float Controller::getValveIncentive(uint8_t controllerIndex, double ascentRate, double altitude, double altitudeSinceLastVentCorrected) {
  float result = 0.0;
  if (controllerIndex == LEGACY_CONTROLLER_INDEX) {
    result = legacyController.getValveIncentive(ascentRate, altitude, altitudeSinceLastVentCorrected);
  }
  return result;
}

/*
 * Function: getBallastIncentive
 * -------------------
 * This function calculates the incentive to actuate the ballast based on a PID
 * feedback controller.
 */
float Controller::getBallastIncentive(uint8_t controllerIndex, double ascentRate, double altitude, double altitudeSinceLastDropCorrected) {
  float result = 0.0;
  if (controllerIndex == LEGACY_CONTROLLER_INDEX) {
    result = legacyController.getValveIncentive(ascentRate, altitude, altitudeSinceLastDropCorrected);
  }
  return result;
}
