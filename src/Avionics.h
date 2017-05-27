/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

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
#include "Simulator.h"
#include "Filters.h"
#include "Hardware.h"
#include "Controller.h"
#include "Payload.h"
#include <SD.h>
#include <GPS.h>
#include <RockBLOCK.h>

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    PCB(),
    sensors(),
    gpsModule(GPS_GATE, GPS_BAUD, EEPROM_GPS, GPS_LOCK_TIME, GPS_QUIT_TIME),
    RBModule(RB_GATE, RB_SLEEP, RB_BAUD, EEPROM_ROCKBLOCK),
    ValMU(PAYLOAD_GATE, EEPROM_PAYLOAD) {
  }
  void    init();
  void    test();
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
  bool    setupSDCard();
  bool    readHistory();

  bool    readData();
  bool    readGPS();
  bool    readPayload();
  bool    simulateData();
  bool    processData();

  bool    calcVitals();
  bool    calcDebug();
  bool    calcIncentives();

  bool    runHeaters();
  bool    runValve();
  bool    runBallast();
  bool    runCutdown();
  bool    runLED();

  bool    sendSATCOMS();
  void    parseCommand(int16_t len);
  void    updateConstant(uint8_t index, float value);
  void    parseManualCommand(bool command);
  void    parseReportCommand(uint8_t command);
  void    parseSensorsCommand(uint8_t command);
  void    parseValveCommand(uint32_t  command);
  void    parseBallastCommand(uint32_t  command);
  void    parseRockBLOCKPowerCommand(bool command);
  void    parseGPSPowerCommand(uint8_t command);
  void    parseHeaterPowerCommand(bool command);
  void    parseHeaterModeCommand(uint8_t command);
  void    parsePayloadPowerCommand(bool command);

  bool    debugState();
  void    setupLog();
  void    printHeader();
  void    logHeader();
  void    logAlert(const char*, bool fatal);
  int16_t compressVariable(float var, float minimum, float maximum, int16_t resolution, int16_t length);
  void    printState();
  bool    logData();
  int16_t compressData();

/*********************************  OBJECTS  **********************************/
  char COMMS_BUFFER[BUFFER_SIZE];
  DataFrame data;
  File dataFile;
  File logFile;
  Hardware PCB;
  Sensors sensors;
  Simulator HITL;
  Filters filter;
  Controller computer;
  GPS gpsModule;
  RockBLOCK RBModule;
  Payload ValMU;
};

#endif
