/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
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
  Payload(uint8_t payloadGatePinNum, uint8_t GPIO_1, uint8_t GPIO_2, uint8_t EEPROMAddressVal) :
    payloadGate(payloadGatePinNum),
    payloadGPIO1(GPIO_1),
    payloadGPIO2(GPIO_2),
    EEPROMAddress(EEPROMAddressVal) {
  }
  bool    init(bool shouldStartup);
/********************************  FUNCTIONS  *********************************/
  void    restart();
  void    shutdown();
  size_t  getMessage();
  char    buf[100]  = {0};
private:
/*********************************  OBJECTS  **********************************/
  uint8_t payloadGate;
  uint8_t payloadGPIO1;
  uint8_t payloadGPIO2;
  uint8_t EEPROMAddress;
};

#endif
