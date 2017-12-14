/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  John Dean | deanjl@stanford.edu
  Ben Newman | blnewman@stanford.edu
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
 * Function: updateInputs
 * -------------------
 * This function updates all the inputs for all of the controllers
 */
void Controller::updateInputs(ControllerInputs allInputs) {
// LEGACY CONTROLLER
  ControllerLegacyInputs legacyInputs;
  legacyInputs.altitude              = allInputs.altitude;
  legacyInputs.altitudeSinceLastVent = allInputs.altitudeSinceLastVent;
  legacyInputs.altitudeSinceLastDrop = allInputs.altitudeSinceLastDrop;
  legacyInputs.ascentRate            = allInputs.ascentRate;
  legacyController.update(legacyInputs);

  SpaghettiController::Input spagInput;
  spagInput.h = allInputs.altitude;
  spagController.update(spagInput);
}

/*
 * Function: getActions
 * -------------------
 * This function gets each controller's actions, adds it to the actions struct
 * and returns the action struct
 */
ControllerActions Controller::getActions() {
  controller_actions.controllerLegacyAction = legacyController.getAction();
  controller_actions.controllerSpagAction   = spagController.getAction();
  return controller_actions;
}

/*
 * Function: getStates
 * -------------------
 * This function gets each controller's state, adds it to the state struct
 * and returns the state struct
 */
ControllerStates Controller::getStates() {
  controller_states.controllerLegacyState = legacyController.getState();
  controller_states.controllerSpagState   = spagController.getState();
  return controller_states;
}
