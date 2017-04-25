/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
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
	sensorInputs  <<  0, 0; //Needs to be initialized to something
	currentState  <<  0, 0;

	currentCovar  <<  99999,   0,
				            0,       99999; //Initially we have zero knowledge of state

	predictionMat <<  1,       0,
					       1/20,       1; //Loop rate goes here

	sensorMat     <<  1,       0, // Actual values. Hope compiler will optimize this out
				            0,       1;

	externalCovar <<  0.00001, 0, //Placeholder values. High ascent rate variance as we have no way of predicting it
					          0,       0.0025;

	sensorCovar   << 60,       0, //Placeholder values
				            0,       4;
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
  //See which sensors are funcitoning correctly
	if (!((MIN_PRESURE < RAW_PRESSURE_1) && (RAW_PRESSURE_1 < MAX_PRESURE))) markFailure(0);
	if (!((MIN_PRESURE < RAW_PRESSURE_2) && (RAW_PRESSURE_2 < MAX_PRESURE))) markFailure(1);
	if (!((MIN_PRESURE < RAW_PRESSURE_3) && (RAW_PRESSURE_3 < MAX_PRESURE))) markFailure(2);
	if (!((MIN_PRESURE < RAW_PRESSURE_4) && (RAW_PRESSURE_4 < MAX_PRESURE))) markFailure(3);
	int numSensorsInRange = 0;
	for (size_t i = 0; i < 4; i++) if (enabledSensors[i]) numSensorsInRange++;
	// Calculate mean of sensors which passed
	double pressIntermediate = 0;
	if (enabledSensors[0]) pressIntermediate += RAW_PRESSURE_1;
	if (enabledSensors[1]) pressIntermediate += RAW_PRESSURE_2;
	if (enabledSensors[2]) pressIntermediate += RAW_PRESSURE_3;
	if (enabledSensors[3]) pressIntermediate += RAW_PRESSURE_4;
	pressIntermediate /= numSensorsInRange;
  // Calculate standard deviation of sensors
	double pressVariance = 0;
	if (enabledSensors[0]) pressVariance += pow(RAW_PRESSURE_1,2);
	if (enabledSensors[1]) pressVariance += pow(RAW_PRESSURE_2,2);
	if (enabledSensors[2]) pressVariance += pow(RAW_PRESSURE_3,2);
	if (enabledSensors[3]) pressVariance += pow(RAW_PRESSURE_4,2);
	pressVariance /= numSensorsInRange;
	pressVariance -= pow(pressIntermediate,2);
  double pressStd = pow(pressVariance,0.5);
  // Test if each sensor is MAX_NUM_STDDEV standard deviations from mean
  if (abs(RAW_PRESSURE_1 - pressIntermediate) > pressStd * MAX_NUM_STDDEV) markFailure(0);
  if (abs(RAW_PRESSURE_2 - pressIntermediate) > pressStd * MAX_NUM_STDDEV) markFailure(1);
  if (abs(RAW_PRESSURE_3 - pressIntermediate) > pressStd * MAX_NUM_STDDEV) markFailure(2);
  if (abs(RAW_PRESSURE_4 - pressIntermediate) > pressStd * MAX_NUM_STDDEV) markFailure(3);
	numSensors = 0;
	for (size_t i = 0; i < 4; i++) if (enabledSensors[i]) numSensors++;
  // If all sensors failed to be within MAX_NUM_STDDEV standard deviations fallback to mean
  if (numSensors == 0) return pressIntermediate;
  // Otherwise use sensors which passed
  double press = 0;
  if (enabledSensors[0]) press += RAW_PRESSURE_1;
  if (enabledSensors[1]) press += RAW_PRESSURE_2;
  if (enabledSensors[2]) press += RAW_PRESSURE_3;
  if (enabledSensors[3]) press += RAW_PRESSURE_4;
  return press / numSensors;
}

/*
 * Function: getNumRejections
 * -------------------
 * This function returns the numer of rejections
 * a specific sensor has encountered.
 */
uint32_t Filters::getNumRejections(uint8_t sensor) {
	return rejectedSensors[sensor - 1];
}

/*
 * Function: kalmanAltitude
 * -------------------
 * This function actually does the kalman.
 */
void Filters::kalmanAltitude(float pressure, float pressureBaseline) {
	storeInputs(pressure, pressureBaseline);
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
 * Function: getKalmanedAscentRate
 * -------------------
 * This function returns the filtered ascent rate.
 */
double Filters::getKalmanedAscentRate() {
  return  currentState(0,0);
}


/*
 * Function: getLowPassAscentRate
 * -------------------
 * This function returns the filtered ascent rate.
 */
double Filters::getLowPassAscentRate() {
    double ascentRateTotal = 0;
    for (int i = 0; i < ASCENT_RATE_BUFFER_SIZE; i++) ascentRateTotal += ASCENT_RATE_BUFFER[i];
    return  ascentRateTotal / ASCENT_RATE_BUFFER_SIZE;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: markFailure
 * -------------------
 * This function marks a specific
 * sensor failure.
 */
void Filters::markFailure(uint8_t sensor){
	enabledSensors[sensor] = false;
	rejectedSensors[sensor]++;
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

  double currentAscentRate = (altitudeCurr - altitudeLast) / ((millis() - ascentRateLast) / 1000.0);
  currentState(0,0) = currentAscentRate; //copy unavenged ascent rate to kalman
  ASCENT_RATE_BUFFER[ascentRateIndex] = currentAscentRate; //copy unavenged ascent rate to lowpass

  ascentRateIndex = (ascentRateIndex + 1) % ASCENT_RATE_BUFFER_SIZE;
  ascentRateLast = millis();
}
