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
#include <Eigen.h>

class Filters {
public:
/**********************************  SETUP  ***********************************/
  bool     init();
/********************************  FUNCTIONS  *********************************/
  void     enableSensors(bool BMP1Enable, bool BMP2Enable, bool BMP3Enable, bool BMP4Enable);
  double   getTemp(double RAW_TEMP_1,double RAW_TEMP_2,double RAW_TEMP_3,double RAW_TEMP_4);
  double   getPressure(double RAW_PRESSURE_1,double RAW_PRESSURE_2,double RAW_PRESSURE_3,double RAW_PRESSURE_4);
  uint32_t getNumRejections(uint8_t sensor);
  void     kalmanAltitude(float pressure, float pressureBaseline);
  double   getKalmanedAltitude();
  double   getKalmanedAscentRate();
  double   getLowPassAscentRate();
private:
/*********************************  HELPERS  **********************************/
  void     markFailure(uint8_t sensor);
  void     storeInputs(float pressure, float pressureBaseline);
/*********************************  OBJECTS  **********************************/
  bool     enabledSensors[4] = {true};
  uint32_t rejectedSensors[4] = {0};
  uint8_t  numSensors;
  uint16_t ascentRateIndex = 0;
  float    ASCENT_RATE_BUFFER[ASCENT_RATE_BUFFER_SIZE];
  double   altitudeCurr;
  double   altitudeLast;
  uint32_t ascentRateLast;
  Eigen::Matrix<double, 2, 1> sensorInputs; // [Ascent_rate , altitude] //z
  Eigen::Matrix<double, 2, 1> currentState; // [Ascent_rate , altitude] //x
  Eigen::Matrix<double, 2, 2> currentCovar; //  P
  Eigen::Matrix<double, 2, 2> predictionMat;//  F
  Eigen::Matrix<double, 2, 2> sensorMat;    //  H
  Eigen::Matrix<double, 2, 2> externalCovar;//  Q
  Eigen::Matrix<double, 2, 2> sensorCovar;  //  R

  float    altitudeBuffer[ALTITUDE_BUFFER_SIZE];
  bool      altitudeErrors[ALTITUDE_BUFFER_SIZE] = {false};

};

#endif
