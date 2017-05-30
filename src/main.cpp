/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
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
1) Valve leak check should not run during ballast motor use and be controllable over RB
2) Variable names from FLOPS
3) all times should be in seconds
4) Readback units should match uplink units
5) Counter variables should have range set to a power of 2 to maximise useful data.
6) remove unessisary downlink variables
7) Double check comms sizes

8) The current battery system current reading is obsolete and has been removed, while external current still remains (edited)
9) Anything to do with GPS current is now full system current _for battery only_ and will have a different voltage ratio for it compared to the other three i2c sensor currents. (You can see now why we need GPS to be low power mode).  This must be sent over RB (edited)
10) We now read in and monitor not only the battery voltage, but also our 5V line voltage, which needs to get reported over RB.
11) Batteries have changed and will now require a different voltage ratio to read, so you have to change that constant in your code

12) GPS low power mode via I2C comms must happen
13) Rockblock sleep must be implemented properly. A sleep counter should be sent over RB.  A way to disable sleep should be implemented.  Restart should be implemented in case all else fails (edited)
14) You need to identify the key reasons for your loop rate, then figure out how to drop it below 25ms (or if you can, 10-15ms).  This is because we will be downclocking the teensy from now on.  This will require you to need an external teensy to print to serial and debug since serial does not work below 24MHz.  After downclocking, we will resume a ~50ms loop rate because of the lower clock.  If you need to investigate other SD card libraries, do it, just make sure this happens or we will weep.  During testing, however, you will not need to downclock.  Only for final verification will we need to downclock.  For testing, stay at 24MHz and try to get <25ms loop rate (edited)

15) A couple of pins changed but I can let you know of those later (edited)
*/
