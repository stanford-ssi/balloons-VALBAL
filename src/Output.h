/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2018
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Keegan Mehall | kmehall@stanford.edu

  File: ValbalOutput.h
  --------------------------
  Interface to PCB hardware.
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
  void            runLED(bool on);
  static void     EEPROMWritelong(uint8_t address, int32_t value);
  static int32_t  EEPROMReadlong(uint8_t address);

private:
};

#endif
