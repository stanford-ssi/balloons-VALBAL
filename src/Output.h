/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2018
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Keegan Mehall | kmehall@stanford.edu

  File: ValbalOutput.h
  --------------------------
  Defines output to PCB Hardware.
  This should include all output from the code to the PCB other than RockBLOCK
*/

#ifndef OUTPUT_H
#define OUTPUT_H

#include "Config.h"
#include <EEPROM.h>
#include <LTC2991.h>

class Output {
public:
/**********************************  SETUP  ***********************************/
  void            init();

/********************************  FUNCTIONS  *********************************/
  //from old hardware class:
  void            runLED(bool on);
  static void     EEPROMWritelong(uint8_t address, int32_t value);
  static int32_t  EEPROMReadlong(uint8_t address);
  //from embedded in the actuators class:
  //valve:
  void            openValve(uint16_t speed);
  void            closeValve(uint16_t speed);//Why are these uint16_t not uint8_t? (copied from old actuators)
  void            stopValve();
  //ballast:
  void            runBallast(bool direction, uint16_t speed);
  void            stopBallast();

  //cutdown
  void            cutdown();
  void            stopCutdown();


private:

};

#endif
