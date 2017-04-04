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
 * Function: init
 * -------------------
 * This function initializes the filter objects.
 */
bool Filters::init() {
  bool sucess = true;

	sensorInputs << 0, 0; //Needs to be initialized to something
	currentState << 0, 0;

	currentCovar << 99999,    0,
				          0,    99999; //Initially we have zero knowledge of state

	predictionMat << 1,    0,
					         1/20, 1; //Loop rate goes here

	sensorMat << 1, 0, // Actual values. Hope compiler will optimize this out
				       0, 1;

	externalCovar << 10, 0, //Placeholder values. High ascent rate variance as we have no way of predicting it
					          0, 1;

	sensorCovar << 1, 0, //Placeholder values
				         0, 1;

  return sucess;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: enableSensors
 * -------------------
 * This function selectivly enables and disables the
 * sensors included in fused calculations.
 */
void Filters::enableSensors(bool BMP1Enable, bool BMP2Enable, bool BMP3Enable, bool BMP4Enable) {
  enabledSensors[0] = BMP1Enable;
  enabledSensors[1] = BMP2Enable;
  enabledSensors[2] = BMP3Enable;
  enabledSensors[3] = BMP4Enable;
  numSensors = 0;
  for (size_t i = 0; i < 4; i++) if (enabledSensors[i]) numSensors++;
}

/*
 * Function: getTemp
 * -------------------
 * This function returns a sensor fused temperature.
 */
double Filters::getTemp(double RAW_TEMP_1, double RAW_TEMP_2, double RAW_TEMP_3, double RAW_TEMP_4) {
  double temp = 0;
  if (enabledSensors[0]) temp += RAW_TEMP_1;
  if (enabledSensors[1]) temp += RAW_TEMP_2;
  if (enabledSensors[2]) temp += RAW_TEMP_3;
  if (enabledSensors[3]) temp += RAW_TEMP_4;
  return temp / numSensors;
}

/*
 * Function: getPressure
 * -------------------
 * This function returns a sensor fused reading.
 */
double Filters::getPressure(double RAW_PRESSURE_1, double RAW_PRESSURE_2, double RAW_PRESSURE_3, double RAW_PRESSURE_4) {
  double press = 0;
  if (enabledSensors[0]) press += RAW_PRESSURE_1;
  if (enabledSensors[1]) press += RAW_PRESSURE_2;
  if (enabledSensors[2]) press += RAW_PRESSURE_3;
  if (enabledSensors[3]) press += RAW_PRESSURE_4;
  return press / numSensors;
}

/*
 * Function: storeInputs
 * -------------------
 * This function returns a higher precision altitude value
 * based on the US 1976 Standard Atmosphere.
 */
void Filters::storeInputs(float pressure, float pressureBaseline) {
  altitudeLast = altitudeCurr;
  if (pressure > 22632.1) altitudeCurr = (44330.7 * (1 - pow(pressure / pressureBaseline, 0.190266)));
  else altitudeCurr =  -6341.73 * log((0.176481 * pressure) / 22632.1);

  currentState(0,1) = altitudeCurr;
  currentState(0,0) = (altitudeCurr - altitudeLast) / ((millis() - ascentRateLast) / 1000.0); //copy unavenged ascent rate
  ascentRateLast = millis();
}

/*
 * Function: kalmanAltitude
 * -------------------
 * This function actually does the kalman
 */
void Filters::kalmanAltitude() {
	// Define Helper Variables
	Eigen::Matrix<double, 2, 1> predictedState;
	Eigen::Matrix<double, 2, 2> predictedCovar;
	Eigen::Matrix<double, 2, 2> K;
	Eigen::Matrix<double, 2, 2> invertPlease;
  // Predict State:
  predictedState = predictionMat * currentState;
  predictedCovar = predictionMat * currentCovar * predictionMat.transpose() + externalCovar;
  // Update state from inputs:
  invertPlease = sensorMat * predictedCovar * sensorMat.transpose() + sensorCovar;
	K = predictedCovar * sensorMat.transpose() * invertPlease.inverse();
	currentState = predictedState + K * (sensorInputs - sensorMat * predictedState);
	currentCovar = predictedCovar - K * sensorMat * predictedCovar;
}

/*
 * Function: returnKalmanedAltitude
 * -------------------
 * This function returns the filtered altitude.
 */
double Filters::getKalmanedAltitude() {
  return  currentState(0,1);
}

/*
 * Function: returnKalmanedAscentRate
 * -------------------
 * This function returns the filtered ascent rate.
 */
double Filters::getKalmanedAscentRate() {
  return  currentState(0,0);
}
