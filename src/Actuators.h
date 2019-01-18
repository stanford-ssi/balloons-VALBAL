/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Jonathan Zwiebel | jzwiebel@stanford.edu

  File: Actuators.h
  --------------------------
  Interface to Physical Actuators.
*/

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "Config.h"
#include <EEPROM.h>
#include <Encoder.h>
#include "spa.h"

class Actuators {
public:
/**********************************  SETUP  ***********************************/
  void     init();

/********************************  FUNCTIONS  *********************************/
  void     updateMechanicalConstants(uint16_t valveMotorSpeedOpenValue, uint16_t valveMotorSpeedCloseValue, uint16_t ballastMotorSpeedValue, uint32_t valveOpeningTimeoutValue, uint32_t valveClosingTimeoutValue);

  void     queueValve(uint32_t  duration, bool real);
  void     queueBallast(uint32_t  duration, bool real);
  void     clearValveQueue();
  void     clearBallastQueue();
  bool     checkValve(float current);
  bool     checkBallast(float current, uint32_t reverseTimeout, uint16_t stallCurrent);
  uint32_t getValveQueue();
  uint32_t getBallastQueue();
  bool     getBallastDirection();
  uint32_t getNumBallastOverCurrents();
  void     clearBallastOverCurrents();

  void     cutDown();

  enum state_t {OPEN, OPENING, CLOSED, CLOSING};
  state_t  valveState = CLOSED;
  state_t  ballastState = CLOSED;

  void pause();
  void play();

  long balenc_count_prev = 0;
  long balenc_sum = 0;
  long valenc_count = 0;

  long val_initial;
  long val_delta;
  bool delta_read = true;

private:
/*********************************  HELPERS  **********************************/
  void     stopValve();
  void     openValve();
  void     closeValve();

  void     stopBallast();
  void     dropBallast(bool direction);

/*********************************  OBJECTS  **********************************/
  uint32_t valveQueue = 0;
  uint32_t valveQueueFake = 0;
  uint32_t ballastQueue = 0;
  uint32_t ballastQueueFake = 0;

  uint16_t valveMotorSpeedOpen = 0;
  uint16_t valveMotorSpeedClose = 0;
  uint16_t ballastMotorSpeed = 0;
  uint32_t valveOpeningTimeout = 0;
  uint32_t valveClosingTimeout = 0;

  uint32_t valveActionStartTime = 0;
  uint32_t ballastActionStartTime = 0;
  uint32_t valveCheckTime = 0;
  uint32_t ballastCheckTime = 0;
  uint32_t ballastStallTime = 0;
  uint32_t ballastDirectionTime = 0;
  uint32_t ballastForceReverseTime = 0;
  uint32_t numBallastOverCurrents = 0;
  float    currentLast = 0;

  bool ballastDirection = false;

  bool paused = false;
};

#endif
