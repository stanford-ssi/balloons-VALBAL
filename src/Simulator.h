/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
  Davy Ragland | dragland@stanford.edu

  File: Simulator.h
  --------------------------
  Client side script to read simuled data to VALBAL
  over Serial for Hardware in the Loop testing.
*/

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Data.h"

class Simulator {
public:
/**********************************  SETUP  ***********************************/
  bool      init();
  DataFrame readData();
/********************************  FUNCTIONS  *********************************/
private:
  size_t    getLine();
/*********************************  OBJECTS  **********************************/
  static const uint16_t UART_BUFFER_SIZE = 1024;
  char buffer[UART_BUFFER_SIZE] = {0};
  DataFrame simulated;
};

#endif
