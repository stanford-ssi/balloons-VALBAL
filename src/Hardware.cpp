/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2018
  Keegan Mehall | kmehall@stanford.edu
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Hardware.cpp
  --------------------------
  Implementation of Hardware.h
*/

#include "Hardware.h"
#include "Utils.h"
#include "Output.h"

void Hardware::init(){
  cutdownState = false;
  valveState = CLOSED;
  ballastState = false;
  ballastDirection = false;
  LEDOn = false;
  output.init();
}

void Hardware::runValveBallast(ValveBallastState state,
                               Hardware::MechanicalConstants mechConsts,
                               float ballastCurrent){
  bool valve = (state == VALVE);
  bool ballast = (state == BALLAST);
  updateValveState(valve, mechConsts);
  updateBallastState(ballast, ballastCurrent, mechConsts);
};

void Hardware::runCutdown(bool start){
  uint32_t currTime = millis();
  if(start){
    cutdownActionEndTime = currTime + CUTDOWN_DURATION;
    output.cutdown();
  }else if(currTime > cutdownActionEndTime){
    output.stopCutdown();
  }
}

/*void Hardware::updateMechanicalConstants(uint16_t valveMotorSpeedOpenValue,
                                         uint16_t valveMotorSpeedCloseValue,
                                         uint16_t ballastMotorSpeedValue,
                                         uint32_t valveOpeningTimeoutValue,
                                         uint32_t valveClosingTimeoutValue,
                                         float    ballastStallCurrentValue){
  valveMotorSpeedOpen = valveMotorSpeedOpenValue;
  valveMotorSpeedClose = valveMotorSpeedCloseValue;
  ballastMotorSpeed = ballastMotorSpeedValue;
  valveOpeningTimeout = valveOpeningTimeoutValue;
  valveClosingTimeout = valveClosingTimeoutValue;
  ballastStallCurrent = ballastStallCurrentValue;
}*/

void Hardware::updateValveState(bool shouldBeOpen,
                                Hardware::MechanicalConstants mechConsts){
  /*  Note: in the way this code is currently written, if the valve is openning
    and commanded to be closed it will still close the full duration even
    though the valve may not be fully open (and the same for closing) */
  uint32_t currTime = millis();

  //if valve is closed or closing and it is supposed to be open:
  if(shouldBeOpen && (valveState == CLOSING || valveState == CLOSED)){
    valveState = OPENING;
    valveActionEndTime = currTime + mechConsts.valveOpeningTimeout;
    output.openValve(mechConsts.valveSpeedOpen);
  }

  //if valve is open or opening and it is supposed to be closed
  if(!shouldBeOpen && (valveState == OPENING || valveState == OPEN)){
    valveState = CLOSING;
    valveActionEndTime = currTime + mechConsts.valveClosingTimeout;
    output.closeValve(mechConsts.valveSpeedClose);
  }

  //if valve is openning or closing and it has already finished
  if((valveState == OPENING || valveState == CLOSING) &&
      currTime > valveActionEndTime){
    output.stopValve();
    if(valveState == OPENING){
      valveState = OPEN;
    }else{
      valveState = CLOSED;
    }
  }
}

void Hardware::reverseBallastDirection(Hardware::MechanicalConstants mechConsts){
  ballastDirectionTime = 0;
  ballastCheckTime = millis();
  ballastDirection = !ballastDirection;
  output.runBallast(ballastDirection, mechConsts.ballastSpeed);
}

void Hardware::updateBallastState(bool shouldBeRunning,
                                  float current,
                                  Hardware::MechanicalConstants mechConsts){
  uint32_t currTime = millis();
  //if it isn't running and it should be:
  if(shouldBeRunning && !ballastState){
    ballastCheckTime = currTime;
    output.runBallast(ballastDirection, mechConsts.ballastSpeed);
  }

  //if it shouldn't be running but is:
  if(!shouldBeRunning && ballastState){
    ballastDirectionTime += currTime - ballastCheckTime;
    output.stopBallast();
  }

  //if it has been running in one direction for too long
  if(ballastState &&
    ballastDirectionTime + currTime - ballastCheckTime
      > mechConsts.ballastReverseInterval){
    reverseBallastDirection(mechConsts);
  }

  //If it isn't stalled
  if(current < mechConsts.ballastStallCurrent){
    ballastStallTime = currTime;
  }

  //if the last time it wasn't stalled was longer ago than the timeout:
  if(currTime - ballastStallTime > BALLAST_STALL_TIMEOUT){
    ballastStallTime = currTime;
    reverseBallastDirection(mechConsts);
  }

  currentLast = current;
}

void Hardware::runLED(bool powerState){
  bool shouldBeOn = (powerState && (uint32_t(millis() / 1000.0) % 2 == 1));
  if(shouldBeOn != LEDOn) output.runLED(shouldBeOn);
}

void Hardware::clearBallastOverCurrents(){
  numBallastOverCurrents = 0;
}
