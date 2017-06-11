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
1) isbd.begin(), transmit, isbd.sleep()
2) wait at least 3 seconds after shutting down or sleeping, to rewaking it
3) A way to disable sleep should be implemented.
4) if isbd.begin() fails or isbd.sleep(), you want to restart the system by fetting it off for 2min and then back on, and set back to no sleep
5) a failure counter should be sent over RB of how many times we restart.
*/
