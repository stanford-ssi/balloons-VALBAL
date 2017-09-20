/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Payload.h
  --------------------------
  Client side script to read payload data
  over Serial from the payload board.
*/

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "Arduino.h"
#include <EEPROM.h>

class Payload {
public:
/**********************************  SETUP  ***********************************/
  Payload(uint8_t payloadGatePinNum, uint8_t GPIO_1, uint8_t GPIO_2, uint8_t DAC, uint8_t EEPROMAddressVal) :
    payloadGate(payloadGatePinNum),
    payloadGPIO1(GPIO_1),
    payloadGPIO2(GPIO_2),
    payloadDAC(DAC),
    EEPROMAddress(EEPROMAddressVal) {
  }
  bool    init(bool shouldStartup);

/********************************  FUNCTIONS  *********************************/
  void    restart();
  void    shutdown();

  bool    setConfig(const char * str, size_t len);
  bool    readyDataFrame();
  bool    addVariable(float var, float minimum, float maximum, int16_t resolution);
  bool    setDataFrame();
  bool    run();

private:
  bool    sendConfig();
  bool    sendDataFrame();
  bool    sendHeartBeat();

/*********************************  OBJECTS  **********************************/
  static const uint8_t SATCOMMS_BUFFER_SIZE = 20;
  static const uint8_t DATA_BUFFER_SIZE = 100;
  char    SATCOMMS_BUFFER[SATCOMMS_BUFFER_SIZE];
  char    DATA_BUFFER[DATA_BUFFER_SIZE];
  int16_t lengthBits = 0;
  int16_t lengthBytes = 0;
  bool    hasNewConfig = false;

  uint8_t payloadGate;
  uint8_t payloadGPIO1;
  uint8_t payloadGPIO2;
  uint8_t payloadDAC;
  uint8_t EEPROMAddress;
};

#endif
