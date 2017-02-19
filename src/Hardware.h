/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Matthew Tan | mratan@stanford.edu

  File: Hardware.h
  --------------------------
  Interface to PCB hardware.
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include "Config.h"
#include <PID_v1.h>


/**********************************  TODO  ************************************/
//Add enablereeboot pin pulled low for 7.1

//maintain queue of actons
// ie if 10 valve commands are sent, do not concurrently actuate,
//but inteligently don't close valve to imidiatly open it again

//also make sure code does not hang, but is guarenteed to terminate

//We only have history of states, no positon feedback from DC motors.
/**********************************  TODO  ************************************/

class Hardware {
public:
/**********************************  SETUP  ***********************************/
  Hardware() :
    pid(&PIDTempVar, &PIDOutVar, &PIDSetVar, 2, 5, 1, DIRECT) {
  }
  void init();
/********************************  FUNCTIONS  *********************************/
  void faultLED();
  void heater(double temp);
  void queueValve(bool force);
  void queueBallast(bool force);
  bool checkValve();
  bool checkBallast();
  void cutDown(bool on);
private:
/*********************************  OBJECTS  **********************************/
  bool     isValveOn = false;
  bool     isBallastOn = false;
  bool     ballastDirection = false;
  double   PIDSetVar;
  double   PIDOutVar;
  double   PIDTempVar;
  PID pid;
};

#endif
