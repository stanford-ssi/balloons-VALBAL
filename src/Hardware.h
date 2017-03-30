/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
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
  Hardware(uint8_t EEPROMAddressVal) :
    EEPROMAddress(EEPROMAddressVal),
    pid(&PIDTempVar, &PIDOutVar, &PIDSetVar, 2, 5, 1, DIRECT) {
  }
  void init();
/********************************  FUNCTIONS  *********************************/
  void runLED(bool on);
  void faultLED();

  bool startUpHeaters(bool shouldStartup);
  void heater(double tempSetpoint, double temp, bool strong, bool weak);
  void turnOffHeaters();
  void setHeaterMode(bool on);

  void queueValve(int duration);
  void queueBallast(int duration);
  void clearValveQueue();
  void clearBallastQueue();
  bool checkValve();
  bool checkBallast();
  bool isValveRunning();
  bool isBallastRunning();

  void cutDown(bool on);

  void writeToEEPROM(uint8_t startByte, uint8_t endByte, int num);
  int  readFromEEPROMAndClear(uint8_t startByte, uint8_t endByte);

private:
/*********************************  HELPERS  **********************************/
  void stopValve();
  void openValve();
  void closeValve();

  void stopBallast();
  void dropBallast(bool direction);

/*********************************  OBJECTS  **********************************/
  enum State {OPEN, OPENING, CLOSED, CLOSING };
  uint8_t  EEPROMAddress;

  // queues represent what Avionics told Hardware to do
  uint64_t valveQueue = 0;
  uint64_t ballastQueue = 0;
  // State's represent the internal state of the Hardware
  State    valveState = CLOSED;
  State    ballastState = CLOSED;
  bool     ballastDirection = false;
  uint64_t valveActionStartTime = 0;
  uint64_t ballastActionStartTime = 0;

  double   PIDSetVar;
  double   PIDOutVar;
  double   PIDTempVar;
  PID pid;
};

#endif
