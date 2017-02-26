/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Ella Hofmann-Coyle | ellahofm@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Controller.h
  --------------------------
  Interface to feedback control algorythm.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Config.h"
#include <math.h>

class Controller {
public:
/**********************************  SETUP  ***********************************/
  bool  init();
/********************************  FUNCTIONS  *********************************/
  void  updateControllerConstants(float incentiveThreshold, float reArmConstant);
  void  updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant);
  void  updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant);
  float getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVent);
  float getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDrop);
private:
/*********************************  OBJECTS  **********************************/
float INCENTIVE_THRESHOLD            =    0;
float RE_ARM_CONSTANT                =    0;
float VALVE_SETPOINT                 =    0;
float VALVE_VELOCITY_CONSTANT        =    0;
float VALVE_ALTITUDE_DIFF_CONSTANT   =    0;
float VALVE_LAST_ACTION_CONSTANT     =    0;
float BALLAST_SETPOINT               =    0;
float BALLAST_VELOCITY_CONSTANT      =    0;
float BALLAST_ALTITUDE_DIFF_CONSTANT =    0;
float BALLAST_LAST_ACTION_CONSTANT   =    0;
};

#endif
