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
  void queueValve(bool force);
  void queueBallast(bool force);
  bool checkValve();
  bool checkBallast();
  void cutDown(bool on);
private:
/*********************************  HELPERS  **********************************/
  void writeToEEPROM(uint8_t startByte, uint8_t endByte, int num);
  int  readFromEEPROMAndClear(uint8_t startByte, uint8_t endByte);
/*********************************  OBJECTS  **********************************/
  bool     isValveOn = false;
  bool     isBallastOn = false;
  bool     ballastDirection = false;
  double   PIDSetVar;
  double   PIDOutVar;
  double   PIDTempVar;
  PID pid;
};

#endif
