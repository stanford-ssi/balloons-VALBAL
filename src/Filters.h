/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.h
  --------------------------
  Interface to guarenteed good derived values.
*/

#ifndef FILTERS_H
#define FILTERS_H

#include "Config.h"
//#include <SD.h>

class Filters {
public:
/**********************************  SETUP  ***********************************/
  bool     init();
/********************************  FUNCTIONS  *********************************/
  void     enableSensors(bool BMP1Enable, bool BMP2Enable, bool BMP3Enable, bool BMP4Enable);
  float    getTemp(float RAW_TEMP_1, float RAW_TEMP_2, float RAW_TEMP_3, float RAW_TEMP_4);
  void     storeData(uint32_t time_stamp, float RAW_PRESSURE_1, float RAW_PRESSURE_2, float RAW_PRESSURE_3, float RAW_PRESSURE_4, float pressureBaselineArg);
  uint32_t getNumRejections(uint8_t sensor);

  float    getAvgCurrentSystem(float current);
  float    getAvgCurrentRB(float current);
  float    getAvgCurrentMotorValve(float current,bool on);
  float    getAvgCurrentMotorBallast(float current,bool on);
  float    getAvgCurrentPayload(float current);

  float    getMinCurrentSystem();
  float    getMaxCurrentSystem();
  float    getMaxCurrentRB();
  float    getMaxCurrentMotorValve();
  float    getMaxCurrentMotorBallast();
  float    getMaxCurrentPayload();

  double   getPressure();
  double   getAltitude();
  double   getAscentRate();
  float    getIncentiveNoise(bool IncludeBMP1, bool IncludeBMP2, bool IncludeBMP3, bool IncludeBMP4);

  void     clearCurrentValues();

private:
/*********************************  HELPERS  **********************************/
  void     consensousCheck();
  void     velocityCheck();
  void     findLastAccepted();
  void     errorCheckAltitudes();
  double   calculateAltitude(float pressure);
  void     markFailure(uint8_t sensor);
/*********************************  OBJECTS  **********************************/
  bool     enabledSensors[4] = {true};
  uint32_t rejectedSensors[4] = {0};
  uint8_t  numSensors;

  float    currentSystemTotal = 0;
  float    currentSystemMax = 0;
  float    currentSystemMin = 10000;
  uint32_t currentSystemCount = 0;
  float    currentRBTotal = 0;
  float    currentRBMax = 0;
  uint32_t currentRBCount = 0;
  float    currentMotorValveTotal = 0;
  float    currentMotorValveMax = 0;
  uint32_t currentMotorValveCount = 0;
  float    currentMotorBallastTotal = 0;
  float    currentMotorBallastMax = 0;
  uint32_t currentMotorBallastCount = 0;
  float    currentPayloadTotal = 0;
  float    currentPayloadMax = 0;
  uint32_t currentPayloadCount = 0;

  float    pressureBaseline;
  float    meanAscentRates[4];
  float    meanAltitudes[4];
  uint16_t altitudeIndex = 0;
  double   sampleTimeSeconds[ALTITUDE_BUFFER_SIZE] = {0};
  float    altitudeBuffer[4][ALTITUDE_BUFFER_SIZE] = {{0}};
  bool     altitudeErrors[4][ALTITUDE_BUFFER_SIZE] = {{false}};

  double   sumX[4] = {0};
  double   sumY[4] = {0};
  double   sumXY[4] = {0};
  double   sumX2[4] = {0};
  int      sampleCount[4] = {ALTITUDE_BUFFER_SIZE,ALTITUDE_BUFFER_SIZE,ALTITUDE_BUFFER_SIZE,ALTITUDE_BUFFER_SIZE};

  bool     firstBUFFER = true;

  float    lastAcceptedAltitudes[4];
  double   lastAcceptedTimes[4];
  double   pressures[4];
  bool     filtered = false;

  // File debugFile;
};

#endif
