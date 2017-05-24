/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu

  File: Simulator.h
  --------------------------
  Client side script to read simuled data to VALBAL
  over Serial for Hardware in the Loop testing.
*/

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Data.h"
#include <SD.h>

class Simulator {
public:
/**********************************  SETUP  ***********************************/
  bool      init();
  DataFrame readData();
  uint32_t  getLoopTime();
/********************************  FUNCTIONS  *********************************/
private:
  void      getLine();
/*********************************  OBJECTS  **********************************/
  static const uint16_t UART_BUFFER_SIZE = 2000;
  char buffer[UART_BUFFER_SIZE] = {0};
  float values[100];
  uint32_t pos = 0;
  uint32_t loopInterval;
  File CSV;
  DataFrame simulated;
};

#endif
