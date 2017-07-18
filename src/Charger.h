/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
  Davy Ragland | dragland@stanford.edu

  File: Charger.h
  --------------------------
  Interface to PCB Charger.
*/

#ifndef CHARGER_H
#define CHARGER_H

#include "Config.h"
#include <PID_v1.h>

class Charger {
public:
/**********************************  SETUP  ***********************************/
  Charger() :
    pid(&PIDTempVar, &PIDOutVar, &PIDSetVar, 90, 9.7, 0, DIRECT) {
  }
  void     init();
/********************************  FUNCTIONS  *********************************/

private:
/*********************************  HELPERS  **********************************/

/*********************************  OBJECTS  **********************************/
  double   PIDSetVar;
  double   PIDOutVar;
  double   PIDTempVar;
  PID pid;
};

#endif
