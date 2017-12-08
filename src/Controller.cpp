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
 * Function: updateConstants
 * -------------------
 * This function updates all the constants for all of the controllers and returns
 * all of the rearm constants.
 */
void Controller::updateConstants(ControllerConstants allConstants) {
  // LEGACY CONTROLLER
  ControllerLegacyConstants legacyConstants = {
    allConstants.valveAltitudeSetpoint,
    allConstants.valveKpConstant,
    allConstants.valveKiConstant,
    allConstants.valveKdConstant,
    allConstants.ballastAltitudeSetpoint,
    allConstants.ballastKpConstant,
    allConstants.ballastKiConstant,
    allConstants.ballastKdConstant,
    allConstants.BallastArmAlt,
    allConstants.incentiveThreshold,
    allConstants.valveVentDuration,
    allConstants.ballastDropDuration
  };
  legacyController.updateConstants(legacyConstants);

  SpaghettiController::Constants spagConstants = {
    allConstants.k,
    allConstants.b_dldt,
    allConstants.v_dldt,
    allConstants.rate_min,
    allConstants.rate_max,
    allConstants.b_tmin,
    allConstants.v_tmin
  };
  spagController.updateConstants(spagConstants);

}

/*
 * Function: updateInputs
 * -------------------
 * This function updates all the inputs for all of the controllers
 */
void Controller::updateInputs(ControllerInputs allInputs) {
// LEGACY CONTROLLER
  ControllerLegacyInputs legacyInputs = {
    allInputs.altitude,
    allInputs.altitudeSinceLastVent,
    allInputs.altitudeSinceLastDrop,
    allInputs.ascentRate
  };
  legacyController.update(legacyInputs);

  SpaghettiController::Input spagInput = {
    allInputs.altitude
  };
  spagController.update(spagInput);
}

/*
 * Function: getActions
 * -------------------
 * This function gets each controller's actions, adds it to the actions struct
 * and returns the action struct
 */
ControllerActions Controller::getActions() {
  // LEGACY CONTROLLER
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
  // LEGACY CONTROLLER
  ALL_CONTROLLER_STATES.controllerLegacyState = legacyController.getState();
  ALL_CONTROLLER_STATES.controllerSpagState   = spagController.getState();
  return ALL_CONTROLLER_STATES;
}
