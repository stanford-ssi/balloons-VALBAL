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
1) remove isbd.begin() from setup, but keep the diags and console setup, and the hw serial begin for rb
2) at the start of your transmission, do isbd.begin()
3) remember that it returns an int and lets you know if it was successful, log that
4) transmit and log the success if transmit as well
5) isbd.sleep() and the end, and log the success of that as well, I used to get ISBD_PROTOCOL_ERROR when I did sleep before…
6) if isbd.begin() fails or sibd.sleep(), you want to restart the system by fetting it off for 2min and then back on
7) if transmit fails but begin() and sleep() are successful, do nothing
8) if you call sleep and you are already asleep, it will let you know that, in that case, something has gone terribly wrong and you need to reset
9) basically anything other than a pure success should result in a full hard power reset
10) A sleep counter should be sent over RB.
11) A way to disable sleep should be implemented.
*/
