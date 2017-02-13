/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: RockBlock.h
  --------------------------
  Interface to Iridium RockBlock satalite communications.
*/

#ifndef RockBlock_H
#define RockBlock_H

#include <IridiumSBD.h>

class RockBLOCK {
public:
/**********************************  SETUP  ***********************************/
  RockBLOCK(uint8_t RB_SLEEP_PIN, uint16_t RB_BAUD_VAL) :
    isbd(Serial3, RB_SLEEP_PIN),
    RB_BAUD(RB_BAUD_VAL) {
  }
  bool    init();
/********************************  FUNCTIONS  *********************************/
  int16_t writeRead(char* buff, uint16_t len);
private:
/*********************************  HELPERS  **********************************/
  void    write(char* buff, uint16_t len);
  void    read(char* buff, uint16_t len);
/*********************************  OBJECTS  **********************************/
  static const uint16_t BUFFER_SIZE = 200;
  uint8_t rxBuffer[BUFFER_SIZE] = {0};
  IridiumSBD isbd;
  uint16_t RB_BAUD;
};

#endif
