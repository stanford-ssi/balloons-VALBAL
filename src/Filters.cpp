/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.cpp
  --------------------------
  Implementation of Filters.h
*/

#include "Filters.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the filter objects
*/
bool Filters::init() {
  bool sucess = true;


  return sucess;
}

/*
  function: getPressure
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Filters::getPressure(double RAW_PRESSURE_1,
                            double RAW_PRESSURE_2,
                            double RAW_PRESSURE_3,
                            double RAW_PRESSURE_4) {
  // Need to add redundancy logic
  return (RAW_PRESSURE_1 + RAW_PRESSURE_2 + RAW_PRESSURE_3 + RAW_PRESSURE_4) / 4;
}

/*
  function: getAltitude
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Filters::getAltitude(double RAW_ALTITUDE_1,
                            double RAW_ALTITUDE_2,
                            double RAW_ALTITUDE_3,
                            double RAW_ALTITUDE_4) {
  altitudeLast = altitudeCurr;

  // Need to add redundancy logic here!
  altitudeCurr = (RAW_ALTITUDE_1 + RAW_ALTITUDE_2 + RAW_ALTITUDE_3 + RAW_ALTITUDE_4) / 4;

  ASCENT_RATE_BUFFER[ascentRateIndex] = (altitudeCurr - altitudeLast) / ((millis() - ascentRateLast) / 1000.0);
  ascentRateLast = millis();
  ascentRateIndex++;
  ascentRateIndex %= BUFFER_SIZE;

  //Add Kalman FIltering here

  return altitudeCurr;
}


/*
  function: getAscentRate
  ---------------------------------
  This function returns the current ascent rate.
*/
double Filters::getAscentRate() {
  float ascentRateTotal = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) ascentRateTotal += ASCENT_RATE_BUFFER[i];
  return  ascentRateTotal / BUFFER_SIZE;
}

/********************************  FUNCTIONS  *********************************/


