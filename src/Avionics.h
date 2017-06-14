/*
  Stanford Student Space Initiative
  Balloons | VALBAL | June 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: avionics.h
  --------------------------
  Primary avionics code.
*/

#ifndef AVIONICS_H
#define AVIONICS_H

#include "Logger.h"
#include "Config.h"
#include "Cutdown.h"
#include "Data.h"
#include "Sensors.h"
//#include "Simulator.h"
#include "Filters.h"
#include "Hardware.h"
#include "Controller.h"
#include "Payload.h"
#include <GPS.h>
#include <RockBLOCK.h>

// regualar min and max is not compatible with vector in std
#define _min(a,b) ((a)<(b)?(a):(b))
#define _max(a,b) ((a)>(b)?(a):(b))

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    PCB(),
    sensors(),
    gpsModule(GPS_GATE, GPS_BAUD, EEPROM_GPS, GPS_LOCK_TIMEOUT, GPS_QUIT_TIMEOUT),
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
  void    parseRockBLOCKModeCommand(bool command);
  void    parseGPSPowerCommand(uint8_t command);
  void    parseHeaterPowerCommand(bool command);
  void    parseHeaterModeCommand(uint8_t command);
  void    parsePayloadPowerCommand(bool command);

  bool    debugState();
  void    setupLog();
  void    printHeader();
  void    logHeader();
  void    alert(const char*, bool fatal);
  int16_t compressVariable(float var, float minimum, float maximum, int16_t resolution, int16_t length);
  void    printState();
  bool    logData();
  int16_t compressData();

/*********************************  OBJECTS  **********************************/
  char COMMS_BUFFER[COMMS_BUFFER_SIZE];
  DataFrame data;
  Logger log;
  Hardware PCB;
  Sensors sensors;
  //Simulator HITL;
  Filters filter;
  Controller computer;
  GPS gpsModule;
  RockBLOCK RBModule;
  Payload ValMU;
};

#endif
