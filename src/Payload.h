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
  float   getCurrentEuler(uint8_t axis);
  float   getAverageEuler(uint8_t axis, uint8_t index);
private:
/*********************************  OBJECTS  **********************************/
  static const uint8_t EULER_BUFFER_SIZE = 100;
  uint8_t  payloadGate;
  uint8_t  EEPROMAddress;
  char     buf[100]  = {0};
  float    values[3] = {0};
  double   eulerXBuf[EULER_BUFFER_SIZE] = {0};
  double   eulerYBuf[EULER_BUFFER_SIZE] = {0};
  double   eulerZBuf[EULER_BUFFER_SIZE] = {0};
  uint8_t  eulerIndex = 0;

  uint32_t eulerAverageStartTime = 0;
  double   eulerXAvgBuf[EULER_BUFFER_SIZE] = {0};
  double   eulerYAvgBuf[EULER_BUFFER_SIZE] = {0};
  double   eulerZAvgBuf[EULER_BUFFER_SIZE] = {0};
  uint8_t  eulerAvgIndex = 0;
};

#endif
