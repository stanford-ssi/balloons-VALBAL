/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Matthew Tan | mratan@stanford.edu

  File: Hardware.h
  --------------------------
  Interface to PCB hardware.
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include "Config.h"
#include <EEPROM.h>
#include <PID_v1.h>

class Hardware {
public:
/**********************************  SETUP  ***********************************/
  Hardware() :
    pid(&PIDTempVar, &PIDOutVar, &PIDSetVar, 2, 5, 1, DIRECT) {
  }
  void init();
/********************************  FUNCTIONS  *********************************/
  void faultLED();

  void heater(double temp);
  void turnOffHeaters();

  void queueValve(int duration);
  void queueBallast(int duration);
  void clearValveQueue();
  void clearBallastQueue();
  bool checkValve();
  bool checkBallast();
  bool isValveRunning();
  bool isBallastRunning();

  void cutDown(bool on);

private:
/*********************************  HELPERS  **********************************/
  void writeToEEPROM(uint8_t startByte, uint8_t endByte, int num);
  int  readFromEEPROMAndClear(uint8_t startByte, uint8_t endByte);

  void stopValve();
  void openValve();
  void closeValve();

  void stopBallast();
  void dropBallast(bool direction);

/*********************************  OBJECTS  **********************************/
  enum State { OPEN, OPENING, CLOSED, CLOSING };

  // queues represent what Avionics told Hardware to do
  int      valveQueue = 0;
  int      ballastQueue = 0;
  // State's represent the internal state of the Hardware
  State    valveState = CLOSED;
  State    ballastState = CLOSED;
  bool     ballastDirection = false;
  int      valveActionStartTime = 0;
  int      ballastActionStartTime = 0;

  double   PIDSetVar;
  double   PIDOutVar;
  double   PIDTempVar;
  PID pid;
};

#endif
