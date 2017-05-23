/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu

  File: Payload.h
  --------------------------
  Client side script to read payload data
  over Serial from the VALMU board.
*/

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "Arduino.h"
#include <EEPROM.h>

class Payload {
public:
/**********************************  SETUP  ***********************************/
  Payload(uint8_t payloadGatePinNum, uint8_t EEPROMAddressVal) :
    payloadGate(payloadGatePinNum),
    EEPROMAddress(EEPROMAddressVal) {
  }
  bool    init(bool shouldStartup);
/********************************  FUNCTIONS  *********************************/
  void    restart();
  void    shutdown();
  void    querrySensors();
  float   getEuler(uint8_t axis);
private:
/*********************************  OBJECTS  **********************************/
  uint8_t payloadGate;
  uint8_t EEPROMAddress;
  char    buf[100]  = {0};
  float   values[3] = {0};
};

#endif
