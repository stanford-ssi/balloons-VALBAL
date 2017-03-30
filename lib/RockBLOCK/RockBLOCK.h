/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: RockBlock.h
  --------------------------
  Interface to Iridium RockBlock satalite communications.
*/

#ifndef RockBlock_H
#define RockBlock_H

#include <IridiumSBD.h>
#include <EEPROM.h>

class RockBLOCK {
public:
/**********************************  SETUP  ***********************************/
  RockBLOCK(uint8_t RB_GatePinNum, uint8_t RB_SleepPinNum, uint16_t RB_BaudVal, uint8_t EEPROMAddressVal) :
    RB_GATE(RB_GatePinNum),
    RB_BAUD(RB_BaudVal),
    EEPROMAddress(EEPROMAddressVal),
    isbd(Serial3, RB_SleepPinNum) {
  }
  bool    init(bool shouldStartup);
/********************************  FUNCTIONS  *********************************/
  void    restart();
  void    shutdown();
  int16_t writeRead(char* buff, uint16_t len);
private:
/*********************************  HELPERS  **********************************/
  void    write(char* buff, uint16_t len);
  void    read(char* buff, uint16_t len);
/*********************************  OBJECTS  **********************************/
  static const uint16_t BUFFER_SIZE = 200;
  uint8_t    rxBuffer[BUFFER_SIZE] = {0};
  uint8_t    RB_GATE;
  uint16_t   RB_BAUD;
  uint8_t    EEPROMAddress;
  IridiumSBD isbd;
};

#endif
