/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.h
  --------------------------
  Interface to guarenteed good derived values.
*/

#ifndef FILTERS_H
#define FILTERS_H

#include "Config.h"
// #include <Eigen.h> // not needed right nows

class Filters {
public:
/**********************************  SETUP  ***********************************/
  bool     init();
/********************************  FUNCTIONS  *********************************/
  void     enableSensors(bool BMP1Enable, bool BMP2Enable, bool BMP3Enable, bool BMP4Enable);
  double   getTemp(double RAW_TEMP_1,double RAW_TEMP_2,double RAW_TEMP_3,double RAW_TEMP_4);
  double   getPressure(double RAW_PRESSURE_1,double RAW_PRESSURE_2,double RAW_PRESSURE_3,double RAW_PRESSURE_4,double pressureBaselineArg);
  uint32_t getNumRejections(uint8_t sensor);
  double   calculateAltitude(double pressure);
  double   getAltitude();
  double   getAscentRate();
  void     consensousCheck();
  void     velocityCheck();
  void     findLastAccepted();
  void     filterAltitudes();
private:
/*********************************  HELPERS  **********************************/
  void     markFailure(uint8_t sensor);
/*********************************  OBJECTS  **********************************/
  bool     enabledSensors[4] = {true};
  uint32_t rejectedSensors[4] = {0};
  uint8_t  numSensors;

  double   pressureBaseline;
  float    meanAscentRates[4];
  float    meanAltitudes[4];
  uint16_t altitudeIndex = 0;
  float    altitudeBuffer[4][ALTITUDE_BUFFER_SIZE];
  bool     altitudeErrors[4][ALTITUDE_BUFFER_SIZE] = {{false}};
  int      numberOfAcceptedSamples[4];
  float    lastAcceptedAltitudes[4];
  float    lastAcceptedTimes[4];
  double   pressures[4];
  bool     sensorsAccepted[4];

};

#endif
