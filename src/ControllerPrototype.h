/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: ControllerPrototype.h
  --------------------------
  Abstract controller specifying what functions all controllers must implement.
*/

#ifndef CONTROLLERPROTOTYPE_H
#define CONTROLLERPROTOTYPE_H

#include "Config.h"

class ControllerPrototype {
public:
  /**********************************  SETUP  ***********************************/
    virtual bool  init() = 0;

  /********************************  FUNCTIONS  *********************************/
    virtual void  updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant) = 0;
    virtual void  updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant) = 0;
    virtual float updateControllerConstants(float BallastArmAlt, float incentiveThreshold) = 0;
    virtual float getAltitudeSinceLastVentCorrected(double altitude, double altitudeSinceLastVent) = 0;
    virtual float getAltitudeSinceLastDropCorrected(double altitude, double altitudeSinceLastDrop) = 0;
    virtual float getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVentCorrected) = 0;
    virtual float getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDropCorrected) = 0;

};

#endif
