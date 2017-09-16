/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Hardware.h
  --------------------------
  Interface to PCB hardware.
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include "Config.h"
#include <EEPROM.h>
#include <LTC2991.h>

class Hardware {
public:
/**********************************  SETUP  ***********************************/
  void            init();
/********************************  FUNCTIONS  *********************************/
  void            runLED(bool on);

  static void     EEPROMWritelong(uint8_t address, int32_t value);
  static int32_t  EEPROMReadlong(uint8_t address);

private:
/*********************************  HELPERS  **********************************/

/*********************************  OBJECTS  **********************************/
};

#endif
