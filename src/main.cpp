/*
  Stanford Student Space Initiative
  Balloons | VALBAL | June 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu
  Jesus Cervantes | cerjesus@stanford.edu
  Matthew Tan | mratan@stanford.edu

  File: main.cpp
  --------------------------
  Flight code for main VALBAL avionics.
*/

#include "Avionics.h"

/***********************************  BOOT  ***********************************/
Avionics VALBAL;
int main(void) {
  VALBAL.init();
  // VALBAL.test();
/***********************************  MAIN  ***********************************/
  while(true) {
    VALBAL.updateState();
    VALBAL.evaluateState();
    VALBAL.actuateState();
    VALBAL.logState();
    VALBAL.sendComms();
    VALBAL.sleep();
  }
  return 0;
}
/*********************************  CALLBACK  *********************************/
bool ISBDCallback() {
  if(VALBAL.finishedSetup()) {
    VALBAL.updateState();
    VALBAL.evaluateState();
    VALBAL.actuateState();
    VALBAL.logState();
    VALBAL.sleep();
  }
  return true;
}

/* TODO
1) Counter variables should have range set to a power of 2 to maximise useful data.
2) remove unessisary downlink variables
3) Double check comms sizes

4) GPS low power mode via I2C comms must happen
5) Rockblock sleep must be implemented properly. A sleep counter should be sent over RB.  A way to disable sleep should be implemented.  Restart should be implemented in case all else fails (edited)
6) You need to identify the key reasons for your loop rate, then figure out how to drop it below 25ms (or if you can, 10-15ms).  This is because we will be downclocking the teensy from now on.  This will require you to need an external teensy to print to serial and debug since serial does not work below 24MHz.  After downclocking, we will resume a ~50ms loop rate because of the lower clock.  If you need to investigate other SD card libraries, do it, just make sure this happens or we will weep.  During testing, however, you will not need to downclock.  Only for final verification will we need to downclock.  For testing, stay at 24MHz and try to get <25ms loop rate (edited)

7) A couple of pins changed but I can let you know of those later (edited)
*/
