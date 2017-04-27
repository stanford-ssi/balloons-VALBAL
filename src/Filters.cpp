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
 * Function: getPressure (renaming to store pressure)
 * -------------------
 * This function returns a bounds checked pressure mean
 */
double Filters::getPressure(double RAW_PRESSURE_1, double RAW_PRESSURE_2, double RAW_PRESSURE_3, double RAW_PRESSURE_4) {
  //See which sensors are funcitoning correctly

    //DECALRE THOSE
    pressure1 = RAW_PRESSURE_1;
    pressure2 = RAW_PRESSURE_2;
    pressure3 = RAW_PRESSURE_3;
    pressure4 = RAW_PRESSURE_4;

	if (!((MIN_PRESURE < RAW_PRESSURE_1) && (RAW_PRESSURE_1 < MAX_PRESURE))) markFailure(0);
	if (!((MIN_PRESURE < RAW_PRESSURE_2) && (RAW_PRESSURE_2 < MAX_PRESURE))) markFailure(1);
	if (!((MIN_PRESURE < RAW_PRESSURE_3) && (RAW_PRESSURE_3 < MAX_PRESURE))) markFailure(2);
	if (!((MIN_PRESURE < RAW_PRESSURE_4) && (RAW_PRESSURE_4 < MAX_PRESURE))) markFailure(3);
	int numSensors = 0;
	for (size_t i = 0; i < 4; i++) if (enabledSensors[i]) numSensors++;
	// Calculate mean of sensors which passed

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
    if(enabledSensors[sensor]) rejectedSensors[sensor]++;
	enabledSensors[sensor] = false;
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

/*
 * Function: getAltitudes
 * -------------------
 * This function returns a higher precision altitude value
 * based on the US 1976 Standard Atmosphere.
 */
void Filters::getAltitudes() {

  float altidude1 = calculateAltitude(pressure1);
  float altidude2 = calculateAltitude(pressure2);
  float altidude3 = calculateAltitude(pressure3);
  float altidude4 = calculateAltitude(pressure4);

  altitudeIndex = (altitudeIndex + 1) % ALTITUDE_BUFFER_SIZE;

  altitudeBuffer[1][altitudeIndex] = altidude1;
  altitudeBuffer[2][altitudeIndex] = altidude2;
  altitudeBuffer[3][altitudeIndex] = altidude3;
  altitudeBuffer[4][altitudeIndex] = altidude4;
  consensousCheck();


}

void consensousCheck(){

    int maxAgreement = 0;
    int maxSensors = 0;
    int minDistance = 0;

    for(int activeSensors = 1; activeSensors<16; activeSensors++){
        int numberOfSensors = 0;
        int numberOfCorrectSensors = 0;
        float meanAltitude = 0;
        float distance = 0;
        for(int sensor = 0; sensor < 4; sensor++){
            if( 1 & (activeSensors>>sensor)){
                numberOfSensors++;
                meanAltitude += altitudeBuffer[sensor][altitudeIndex];
            }
        }
        meanAltitude /= numberOfSensors;
        for(int sensor = 0; sensor < 4; sensor++){
            if(1 & (activeSensors>>sensor)){
                distance += pow(altitudeBuffer[sensor][altitudeIndex] - meanAltitude,2);
                if(fabs(altitudeBuffer[sensor][altitudeIndex] - meanAltitude) < MAX_CONSENSUS_DEVIATION) numberOfCorrectSensors +=1;
            }
        }
        if(numberOfCorrectSensors > maxSensors || (numberOfCorrectSensors == maxSensors && distance < minDistance)){
            maxAgreement = activeSensors;
            maxSensors = numberOfSensors;
            minDistance = distance;
        }
    }

    for(int sensor = 0; sensor < 4; sensor++){
        if( 1 & (maxSensors>>sensor)){
            markFailure(sensor);
        }
    }


}


/*
 * Function: calculateAltitude
 * -------------------
 * This function returns a higher precision altitude value
 * based on the US 1976 Standard Atmosphere.
 */
float Filters::calculateAltitude(double pressure, float pressureBaseline) {
  float calculatedAltitude;
  if (pressure > 22632.1) calculatedAltitude = (44330.7 * (1 - pow(pressure / pressureBaseline, 0.190266)));
  else calculatedAltitude =  -6341.73 * log((0.176481 * pressure) / 22632.1);

  return calculatedAltitude;
}
