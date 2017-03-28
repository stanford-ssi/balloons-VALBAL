/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
  Davy Ragland | dragland@stanford.edu

  File: avionics.h
  --------------------------
  Primary avionics code.
*/

#ifndef AVIONICS_H
#define AVIONICS_H

#include "Config.h"
#include "Cutdown.h"
#include "Data.h"
#include "Sensors.h"
#include "Filters.h"
#include "Hardware.h"
#include "Controller.h"
#include <SD.h>
#include <GPS.h>
#include <RockBLOCK.h>

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    PCB(EEPROM_HEATER),
    sensors(),
    gpsModule(GPS_ENABLE, GPS_BAUD, EEPROM_GPS, GPS_LOCK_TIME),
    RBModule(RB_GATE, RB_SLEEP, RB_BAUD, EEPROM_ROCKBLOCK) {
  }
  void    init();
/********************************  FUNCTIONS  *********************************/
  void    updateState();
  void    evaluateState();
  void    actuateState();
  void    logState();
  void    sendComms();
  void    sleep();
  bool    finishedSetup();

private:
/*********************************  HELPERS  **********************************/
  bool    readHistory();
  bool    readData();
  bool    processData();
  bool    runHeaters();
  bool    runValve();
  bool    runBallast();
  bool    runCutdown();
  bool    sendSATCOMS();
  void    parseCommand(int16_t len);
  bool    calcVitals();
  bool    calcDebug();
  bool    calcIncentives();
  bool    calcCutdown();
  bool    debugState();
  void    setupLog();
  void    printHeader();
  void    logHeader();
  void    logAlert(const char*, bool fatal);
  int16_t compressVariable(float var, float minimum, float maximum, int16_t resolution, int16_t length);
  void    printState();
  bool    logData();
  int16_t compressData();
  void    updateConstant(uint8_t index, float value);
  void    parseAvionicsModeCommand(int command);
  void    parseRockBlockCommand(bool command);
  void    parseGPSCommand(int command);
  void    parseHeaterCommand(bool command);
  void    parseHeaterModeCommand(int command);
/*********************************  OBJECTS  **********************************/
  char COMMS_BUFFER[BUFFER_SIZE];
  DataFrame data;
  File dataFile;
  File logFile;
  Hardware PCB;
  Sensors sensors;
  Filters filter;
  Controller computer;
  GPS gpsModule;
  RockBLOCK RBModule;
};

#endif
