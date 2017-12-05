/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Controller.h
  --------------------------
  Interface to feedback control algorithms.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Config.h"
#include "ControllerLegacy.h"

class Controller {
public:
/**********************************  SETUP  ***********************************/
  bool  init();

/********************************  FUNCTIONS  *********************************/
  void  updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant);
  void  updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant);
  float updateControllerConstants(uint8_t controllerIndex, float BallastArmAlt, float incentiveThreshold);
  float getAltitudeSinceLastVentCorrected(uint8_t controllerIndex, double altitude, double altitudeSinceLastVent);
  float getAltitudeSinceLastDropCorrected(uint8_t controllerIndex, double altitude, double altitudeSinceLastDrop);
  float getValveIncentive(uint8_t controllerIndex, double ascentRate, double altitude, double altitudeSinceLastVentCorrected);
  float getBallastIncentive(uint8_t controllerIndex, double ascentRate, double altitude, double altitudeSinceLastDropCorrected);

private:
/*********************************  OBJECTS  **********************************/
  ControllerLegacy legacyController;

};

#endif
