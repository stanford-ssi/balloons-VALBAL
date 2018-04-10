/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2018
  Keegan Mehall | kmehall@stanford.edu

  File: Hardware.h
  ----------------
  Hardware specific interface between the avionics and output classes
  The avionics class needs to repeatedly call runValveBallast, runCutdown, and
    runLED to keep their states correct. These could also be combined into a
    single method.
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include "Config.h"
#include "Output.h"
#include "Utils.h"

class Hardware{
public:
  void init();
  typedef struct{
    uint16_t valveSpeedOpen;
    uint16_t valveSpeedClose;
    uint32_t valveOpeningTimeout;
    uint32_t valveClosingTimeout;
    uint16_t ballastSpeed;
    float    ballastStallCurrent;
    uint32_t ballastReverseInterval;
  } MechanicalConstants;
  void runValveBallast(ValveBallastState state,
                       Hardware::MechanicalConstants mechConsts,
                       float ballastCurrent);
  void runCutdown(bool start);
  void clearBallastOverCurrents();

/********************************  FUNCTIONS  *********************************/
  void            runLED(bool on);

private:
  void updateValveState(bool shouldBeOpen, MechanicalConstants mechConsts);
  void updateBallastState(bool shouldBeRunning,
                          float current,
                          MechanicalConstants mechConsts);
  void reverseBallastDirection(MechanicalConstants mechConsts);

  Output output;

  enum valveState_t {OPEN, OPENING, CLOSED, CLOSING};

  bool           ballastState;
  valveState_t   valveState;
  bool           cutdownState;
  bool           LEDOn;
  bool           ballastDirection;


  uint32_t valveActionEndTime = 0;
  uint32_t ballastActionStartTime = 0;
  uint32_t cutdownActionEndTime = 0;
  uint32_t valveCheckTime = 0;
  uint32_t ballastCheckTime = 0;
  uint32_t ballastStallTime = 0;
  uint32_t ballastDirectionTime = 0;
  uint32_t numBallastOverCurrents = 0;
  float    currentLast = 0;
};

#endif
