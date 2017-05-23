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

//Debugging only
#include <SD.h>

class Filters {
public:
  File debugFile;
/**********************************  SETUP  ***********************************/
  bool     init();
/********************************  FUNCTIONS  *********************************/
  void     enableSensors(bool BMP1Enable, bool BMP2Enable, bool BMP3Enable, bool BMP4Enable);
  double   getTemp(double RAW_TEMP_1,double RAW_TEMP_2,double RAW_TEMP_3,double RAW_TEMP_4);
  void     storeData(uint32_t time_stamp, double RAW_PRESSURE_1,double RAW_PRESSURE_2,double RAW_PRESSURE_3,double RAW_PRESSURE_4,double pressureBaselineArg);
  uint32_t getNumRejections(uint8_t sensor);

  double   getAverageCurrentSystem(double current);
  double   getAverageCurrentGPS(double current);
  double   getAverageCurrentRB(double current);
  double   getAverageCurrentMotors(double current,bool on);
  double   getAverageCurrentPayload(double current);

  double   getPressure();
  double   getAltitude();
  double   getAscentRate();
  float    getIncentiveNoise(bool IncludeBMP1, bool IncludeBMP2, bool IncludeBMP3, bool IncludeBMP4);

  void     clearAverages();

private:
/*********************************  HELPERS  **********************************/
  void     consensousCheck();
  void     velocityCheck();
  void     findLastAccepted();
  void     errorCheckAltitudes();
  double   calculateAltitude(double pressure);
  void     markFailure(uint8_t sensor);
/*********************************  OBJECTS  **********************************/
  bool     enabledSensors[4] = {true};
  uint32_t rejectedSensors[4] = {0};
  uint8_t  numSensors;

  double   currentSystemTotal = 0;
  uint32_t currentSystemCount = 0;
  double   currentGPSTotal = 0;
  uint32_t currentGPSCount = 0;
  double   currentRBTotal = 0;
  uint32_t currentRBCount = 0;
  double   currentMotorsTotal = 0;
  uint32_t currentMotorsCount = 0;
  double   currentPayloadTotal = 0;
  uint32_t currentPayloadCount = 0;

  double   pressureBaseline;
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

};

#endif
