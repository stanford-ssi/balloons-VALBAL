/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: GPS.h
  --------------------------
  Interface to Ublox NEO-M8Q GPS module.
*/

#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
#include <EEPROM.h>

class GPS {
public:
/**********************************  SETUP  ***********************************/
  GPS(uint8_t GPS_EnablePinNum, uint16_t GPS_BaudVal, uint16_t GPS_LockTime) :
    GPS_ENABLE_PIN(GPS_EnablePinNum),
    GPS_BAUD(GPS_BaudVal),
    GPS_LOCK_TIME(GPS_LockTime) {
  }
  bool     init();
/********************************  FUNCTIONS  *********************************/
  float    getLatitude();
  float    getLongitude();
  double   getAltitude();
  double   getSpeed();
  double   getCourse();
  uint32_t getSats();
  void     smartDelay(uint64_t ms);
private:
/*********************************  HELPERS  **********************************/
  void     setFlightMode(uint16_t GPS_LOCK_TIME);
  void     sendUBX(uint8_t* MSG, uint8_t len);
  bool     getUBX_ACK(uint8_t* MSG);
/*********************************  OBJECTS  **********************************/
  uint8_t  GPS_ENABLE_PIN;
  uint16_t GPS_BAUD;
  uint16_t GPS_LOCK_TIME;
  TinyGPSPlus tinygps;
};

#endif
