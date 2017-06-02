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
1) GPS low power mode via I2C comms must happen
2) Rockblock sleep must be implemented properly. A sleep counter should be sent over RB.  A way to disable sleep should be implemented.  Restart should be implemented in case all else fails (edited)
*/

// #include "Config.h"
// #include <GPS.h>
// #include "Sensors.h"
//
// int main(void) {
//   GPS gpsModule(GPS_GATE, GPS_BAUD, EEPROM_GPS, GPS_LOCK_TIMEOUT, GPS_QUIT_TIMEOUT);
//   Sensors sensors;
//   gpsModule.init(true);
//   sensors.init();
//
//   float currents[255] = {100};
//   uint8_t currentIndex = 0;
//   while(true){
//     gpsModule.smartDelay(50);
//
//     float current = sensors.getCurrentTotal() / 2.0;
//     currents[currentIndex] = current;
//     currentIndex++;
//     float currentAverage = 0;
//     for(size_t i = 0; i < 255; i++) currentAverage += currents[i];
//
//     Serial.print("CURRENT :");
//     Serial.print(current);
//     Serial.print(" CURRENT AVERAGE :");
//     Serial.print(currentAverage / 255);
//     Serial.print(" LATITUDDE: ");
//     Serial.print(gpsModule.getLatitude());
//     Serial.print(" lONGITUDE: ");
//     Serial.print(gpsModule.getLongitude());
//     Serial.print(" NUM SATS: ");
//     Serial.print(gpsModule.getSats());
//     Serial.print('\n');
//   }
//   return 0;
// }
