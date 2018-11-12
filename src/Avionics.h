/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
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
#include "CurrentSensor.h"
//#include "Simulator.h"
#include "Filters.h"
#include "Charger.h"
#include "Hardware.h"
#include "Actuators.h"
#include "Controller.h"
#include "Radio.h"
#include "Heater.h"
#include <GPS.h>
#include <RockBLOCK.h>
#include <Adafruit_NeoPixel.h>
#include "Utils.h"

// #define JANKSHITL
//#define SERIALSHITL
//#define SERIALMONITOR
//#define SERIALSHITL_LEN 32



// regualar min and max is not compatible with vector in std
#define _min(a,b) ((a)<(b)?(a):(b))
#define _max(a,b) ((a)>(b)?(a):(b))

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    #ifdef JANKSHITL
    //stepsim({{1, -1.999969998200029, 0.999970000449996},{0, 1.124988749873439e-9, 1.124977500042189e-9}}),
    //tempsim({{1, -1.999999750000005, 1.000000000000000},{0, 0.124999997395833e-06, 0.124999997395833e-06}}),
    #endif
    superCap(),
    PCB(),
    actuator(),
    sensors(),
    gpsModule(GPS_GATE, GPS_BAUD, EEPROM_GPS, GPS_LOCK_TIMEOUT, GPS_QUIT_TIMEOUT),
    RBModule(RB_GATE, RB_SLEEP, RB_BAUD, EEPROM_ROCKBLOCK),
    radio(PAYLOAD_GATE, PAYLOAD_GPIO_1, PAYLOAD_GPIO_2, PAYLOAD_DAC, EEPROM_PAYLOAD),
    op_filter({{1.0627905195293135L, -1.9960534568565431L, 0.9372094804706866L}, {0.0009866357858642205L, 0.001973271571728441L, 0.0009866357858642205L}}),
    op_vref_filter({{1.0627905195293135L, -1.9960534568565431L, 0.9372094804706866L}, {0.0009866357858642205L, 0.001973271571728441L, 0.0009866357858642205L}}),
    sunsetPredictor(),
		pixels(1, 25, NEO_GRB + NEO_KHZ800)
    {
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
  bool    setup5VLine();

  bool    readData();
  bool    readGPS();
  bool    simulateData();
  bool    processData();

  bool    calcVitals();
  bool    calcDebug();
  bool    calcIncentives();

  bool    runCharger();
  bool    runValve();
  bool    runBallast();
  bool    runCutdown();
  bool    runLED();
  bool    runRadio();

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
  void    parseResistorPowerCommand(uint8_t command);
  void    parsePayloadPowerCommand(bool command);
  void    parseRadioPowerCommand(bool command);
  void    parseRockBLOCKModeCommand(bool command);  bool    debugState();
  void    setupLog();
  void    logHeader();
  void    alert(const char*, bool fatal);
  void    clearVariables();
  int16_t compressVariable(float var, float minimum, float maximum, int16_t resolution, int16_t length);
  void    printState();
  bool    logData();
  int16_t compressData();
  void    shitlUpdate();
  void    serialMonitorUpdate();
  int     numExecReset();

/*********************************  OBJECTS  **********************************/

  #ifdef JANKSHITL
  //Biquad stepsim;
  //Biquad tempsim;
  #endif

  int numExecNow = 0;

  char COMMS_BUFFER[COMMS_BUFFER_SIZE];
  DataFrame data;
  Logger log;
  Charger superCap;
  Hardware PCB;
  Actuators actuator;
  Sensors sensors;
  CurrentSensor currentSensor;
  //Simulator HITL;
  Filters filter;
  Controller computer;
  GPS gpsModule;
  RockBLOCK RBModule;
  Radio radio;

  Biquad op_filter;
  Biquad op_vref_filter;

  LasagnaController lasController;

  Heater heater;

	Adafruit_NeoPixel pixels;

  uint32_t lastSunsetUpdate = 0;
  SunsetPredictor sunsetPredictor;
  unsigned int sun_ctr = 0;
  void runHeaters();
  void rumAndCoke();
  bool checkInCuba();
  void timedCutdown();
  void runDeadMansSwitch();
  void updateSunValues();
};


#endif
