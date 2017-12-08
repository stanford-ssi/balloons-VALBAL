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

typedef struct {
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
} ControllerLegacyConstants;

typedef struct {
  double altitude;
  double altitudeSinceLastVent;
  double altitudeSinceLastDrop;
  double ascentRate;
} ControllerLegacyInputs;

typedef struct {
  float valveIncentive;
  float ballastIncentive;

  float altitudeSinceLastVentCorrected;
  float altitudeSinceLastDropCorrected;
  float altitude;
  float ascentRate;

  float reArmConstant;
} ControllerLegacyState;

class ControllerLegacy {//: public ControllerPrototype {
public:
  virtual bool  init();

/********************************  FUNCTIONS  *********************************/
  virtual void updateConstants(ControllerLegacyConstants constants);
  virtual void update(ControllerLegacyInputs inputs);
  virtual int32_t getAction();
  virtual ControllerLegacyState getState();

private:
/********************************  FUNCTIONS  *********************************/
  virtual void  updateValveConstants(float valveAltitudeSetpoint, float valveKpConstant, float valveKiConstant, float valveKdConstant);
  virtual void  updateBallastConstants(float ballastAltitudeSetpoint, float ballastKpConstant, float ballastKiConstant, float ballastKdConstant);
  virtual float updateControllerConstants(float BallastArmAlt, float incentiveThreshold);
  virtual float getAltitudeSinceLastVentCorrected(double altitude, double altitudeSinceLastVent);
  virtual float getAltitudeSinceLastDropCorrected(double altitude, double altitudeSinceLastDrop);
  virtual float getValveIncentive(double ascentRate, double altitude, double altitudeSinceLastVentCorrected);
  virtual float getBallastIncentive(double ascentRate, double altitude, double altitudeSinceLastDropCorrected);

/*********************************  OBJECTS  **********************************/
  bool  firstBallastDropped            = false;
  ControllerLegacyConstants CONSTANTS  = {
    0,// valveAltitudeSetpoint;
    0,// valveKpConstant;
    0,// valveKiConstant;
    0,// valveKdConstant;
    0,// ballastAltitudeSetpoint;
    0,// ballastKpConstant;
    0,// ballastKiConstant;
    0,// ballastKdConstant;
    0,// BallastArmAlt;
    0// incentiveThreshold;
  };
  ControllerLegacyState STATE          = {
    0, //valveIncentive
    0, //ballastIncentive
    0, //altitudeSinceLastVentCorrected
    0, //altitudeSinceLastDropCorrected
    0
  };
};

#endif
