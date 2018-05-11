/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Radio.h
  --------------------------
  Client side script to read payload data
  over Serial from the payload board.
*/

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "Arduino.h"
#include <EEPROM.h>
#include "RadioInterface.h"

class Radio {
public:
/**********************************  SETUP  ***********************************/
  Radio(uint8_t payloadGatePinNum, uint8_t GPIO_1, uint8_t GPIO_2, uint8_t DAC, uint8_t EEPROMAddressVal) :
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

  bool    readyDataFrame();
  bool    addVariable(float var, float minimum, float maximum, int16_t resolution);
  bool    setDataFrame();
  bool    run();
  bool    send_message(vb_rf_message* msg);

  bool    hasNewConfig = false;
  float theLatitude = 0.0;
  float theLongitude = 0.0;

private:
  bool    sendDataFrame();

/*********************************  OBJECTS  **********************************/
  static const uint8_t SATCOMMS_BUFFER_SIZE = 20;
  static const uint8_t DATA_BUFFER_SIZE = 100;
  char     SATCOMMS_BUFFER[SATCOMMS_BUFFER_SIZE];
  char     DATA_BUFFER[DATA_BUFFER_SIZE];
  int16_t  lengthBits   = 0;
  int16_t  lengthBytes  = 0;
  int      heartBeatViolations = 0;
  uint32_t lastStartupTime = 0;
  bool     sent = false;

  uint8_t payloadGate;
  uint8_t payloadGPIO1;
  uint8_t payloadGPIO2;
  uint8_t payloadDAC;
  uint8_t EEPROMAddress;
};

#endif
