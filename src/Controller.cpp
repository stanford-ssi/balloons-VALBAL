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
 * Function: updateConstants
 * -------------------
 * This function updates all the constants for all of the controllers and returns
 * all of the rearm constants.
 */
void Controller::updateConstants(ControllerConstants allConstants) {
  // LEGACY CONTROLLER
  ControllerLegacyConstants legacyConstants;
  legacyConstants.valveAltitudeSetpoint   = allConstants.valveAltitudeSetpoint;
  legacyConstants.valveKpConstant         = allConstants.valveKpConstant;
  legacyConstants.valveKiConstant         = allConstants.valveKiConstant;
  legacyConstants.valveKdConstant         = allConstants.valveKdConstant;
  legacyConstants.ballastAltitudeSetpoint = allConstants.ballastAltitudeSetpoint;
  legacyConstants.ballastKpConstant       = allConstants.ballastKpConstant;
  legacyConstants.ballastKiConstant       = allConstants.ballastKiConstant;
  legacyConstants.ballastKdConstant       = allConstants.ballastKdConstant;
  legacyConstants.BallastArmAlt           = allConstants.BallastArmAlt;
  legacyConstants.incentiveThreshold      = allConstants.incentiveThreshold;
  legacyConstants.valveVentDuration       = allConstants.valveVentDuration;
  legacyConstants.ballastDropDuration     = allConstants.ballastDropDuration;
  legacyController.updateConstants(legacyConstants);

  SpaghettiController::Constants spagConstants;
  spagConstants.k          = allConstants.k;
  spagConstants.b_dldt     = allConstants.b_dldt;
  spagConstants.b_dldt     = allConstants.b_dldt;
  spagConstants.rate_min   = allConstants.rate_min;
  spagConstants.rate_max   = allConstants.rate_max;
  spagConstants.b_tmin     = allConstants.b_tmin;
  spagConstants.v_tmin     = allConstants.v_tmin;
  spagConstants.h_cmd      = allConstants.h_cmd;
  spagController.updateConstants(spagConstants);

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
  ALL_CONTROLLER_ACTIONS.controllerLegacyAction = legacyController.getAction();
  ALL_CONTROLLER_ACTIONS.controllerSpagAction   = spagController.getAction();
  return ALL_CONTROLLER_ACTIONS;
}

/*
 * Function: getStates
 * -------------------
 * This function gets each controller's state, adds it to the state struct
 * and returns the state struct
 */
ControllerStates Controller::getStates() {
  ALL_CONTROLLER_STATES.controllerLegacyState = legacyController.getState();
  ALL_CONTROLLER_STATES.controllerSpagState   = spagController.getState();
  return ALL_CONTROLLER_STATES;
}
