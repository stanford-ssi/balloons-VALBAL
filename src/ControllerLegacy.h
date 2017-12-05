/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: ControllerLegacy.h
  --------------------------
  The legacy PID-like VALBAL controller
*/

#ifndef CONTROLLERLEGACY_H
#define CONTROLLERLEGACY_H

#include "Config.h"
#include "ControllerPrototype.h"

class ControllerLegacy : public ControllerPrototype {
public:
  virtual bool  init();

/********************************  FUNCTIONS  *********************************/
  virtual void  updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant);
  virtual void  updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant);
  virtual float updateControllerConstants(float BallastArmAlt, float incentiveThreshold);
  virtual float getAltitudeSinceLastVentCorrected(double altitude, double altitudeSinceLastVent);
  virtual float getAltitudeSinceLastDropCorrected(double altitude, double altitudeSinceLastDrop);
  virtual float getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVentCorrected);
  virtual float getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDropCorrected);

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
};

#endif
