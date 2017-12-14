/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  John Dean | deanjl@stanford.edu
  Ben Newman | blnewman@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Controller.h
  --------------------------
  Interface to feedback control algorithms.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Config.h"
#include "ControllerLegacy.h"
#include "SpaghettiController.h"

// INPUT STRUCTS
typedef struct {
  // LEGACY CONTROLLER CONSTANTS
  float valveAltitudeSetpoint;
  float valveKpConstant;
  float valveKiConstant;
  float valveKdConstant;
  float ballastAltitudeSetpoint;
  float ballastKpConstant;
  float ballastKiConstant;
  float ballastKdConstant;
  float BallastArmAlt;
  float incentiveThreshold;
  uint32_t valveVentDuration;
  uint32_t ballastDropDuration;
  // SPAGHETTI CONTROLLER CONSTANTS
  float freq;
  float k;                  // gain modifier
  float b_dldt;             // balast dl/dt (kg/s)
  float v_dldt;             // valve dl/dt (kg/s)
  float rate_min;           // min dl/dt rate threshold (kg/s)
  float rate_max;           // max dl/dt rate threshold (kg/s)
  float b_tmin;               // minimum ballast event time
  float v_tmin;
  float h_cmd;
} ControllerConstants;

typedef struct {
  double altitude;
  double altitudeSinceLastVent;
  double altitudeSinceLastDrop;
  double ascentRate;
} ControllerInputs;


// RETURN STRUCTS
typedef struct {
  ControllerLegacyState     controllerLegacyState;
  SpaghettiController::State  controllerSpagState;
} ControllerStates;

typedef struct {
  int32_t controllerLegacyAction;
  int32_t controllerSpagAction;
} ControllerActions;

class Controller {
public:
  ControllerActions controller_actions;
  ControllerStates controller_states;

/**********************************  SETUP  ***********************************/
  bool  init();

/********************************  FUNCTIONS  *********************************/
  void  updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant);
  void  updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant);
  float updateControllerConstants(float BallastArmAlt, float incentiveThreshold);  void updateInputs(ControllerInputs allInputs);
  ControllerActions getActions();
  ControllerStates getStates();

private:
/*********************************  OBJECTS  **********************************/
float RE_ARM_CONSTANT                =     0;
float BALLAST_ARM_ALT                =     0;
float VALVE_SETPOINT                 =     0;
float VALVE_VELOCITY_CONSTANT        =     0;
float VALVE_ALTITUDE_DIFF_CONSTANT   =     0;
float VALVE_LAST_ACTION_CONSTANT     =     0;
float BALLAST_SETPOINT               =     0;
float BALLAST_VELOCITY_CONSTANT      =     0;
float BALLAST_ALTITUDE_DIFF_CONSTANT =     0;
float BALLAST_LAST_ACTION_CONSTANT   =     0;
bool  firstBallastDropped            = false;

ControllerLegacy legacyController;
SpaghettiController spagController;

};

#endif
