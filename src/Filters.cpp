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
    findLastAccepted();
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
 * This function returns a bounds checked pressure mean
 */
double Filters::getPressure(double RAW_PRESSURE_1, double RAW_PRESSURE_2, double RAW_PRESSURE_3, double RAW_PRESSURE_4) {
  //See which sensors are funcitoning correctly

    pressures[1] = RAW_PRESSURE_1;
    pressures[2] = RAW_PRESSURE_2;
    pressures[3] = RAW_PRESSURE_3;
    pressures[4] = RAW_PRESSURE_4;

	for(int i = 0; i<4;i++) if(!((MIN_PRESURE < pressures[i]) && (pressures[i] < MAX_PRESURE))) markFailure(i);

	int numSensors = 0;
	for (size_t i = 0; i < 4; i++) if (enabledSensors[i]) numSensors++;
	// Calculate mean of sensors which passed

	double press = 0;
	for(int i = 0; i<4;i++) if (enabledSensors[i]) press += pressures[i];

  return press / numSensors;
}

/***************************  GET FUNCTIONS  **********************************/

/*
 * Function: getAltitude
 * -------------------
 * This function returns an error checked and smoothed
 * altitude value
 */
void Filters::getAltitude() {

  filterAltitudes();

  float meanAltitude = 0;
  int acceptedStreams = 0;

  for(int i = 0; i<4;i++){
      float altitudesSum =0;
      bool sensorAccepted = true;

      for(int t = 0; t<ALTITUDE_BUFFER_SIZE;t++){
          altitudesSum += altitudeBuffer[i][t];
          sensorAccepted = sensorAccepted && !altitudeErrors[i][t];
      }

      if(sensorAccepted){
          meanAltitude += altitudesSum / ALTITUDE_BUFFER_SIZE
          acceptedStreams++;
      }
  }
  return meanAltitude / acceptedStreams;
}

/*
 * Function: getAscentRate
 * -------------------
 * This function returns the filtered ascent rate.
 */
double Filters::getAscentRate() {
    double ascentRateTotal = 0;
    for (int i = 0; i < ASCENT_RATE_BUFFER_SIZE; i++) ascentRateTotal += ASCENT_RATE_BUFFER[i];
    return  ascentRateTotal / ASCENT_RATE_BUFFER_SIZE;
}


/********************************  CHECKERS  **********************************/


/*
 * Function: filterAltitudes
 * -------------------
 * This function returns a higher precision altitude value
 * based on the US 1976 Standard Atmosphere.
 */
void Filters::filterAltitudes() {
  altitudeIndex = (altitudeIndex + 1) % ALTITUDE_BUFFER_SIZE;
  for(int i = 0; i<4;i++) altitudeBuffer[i][altitudeIndex] = calculateAltitude(pressures[i]);

  consensousCheck();
  velocityCheck();
  findLastAccepted();
}

/*
 * Function: findLastAccepted
 * -------------------
 * This function checkes if velocity since last accepted
 * altitude is within an acceptible range
 */
void Filters::findLastAccepted() {

    for(int i = 0; i<4;i++){
        if (enabledSensors[i]){
            lastAcceptedAltitudes = altitudeBuffer[i][altitudeIndex];
            lastAcceptedTimes = millis();
        }
    }
}


/*
 * Function: velocityCheck
 * -------------------
 * This function checkes if velocity since last accepted
 * altitude is within an acceptible range
 */
void Filters::velocityCheck() {
    for(int i = 0; i<4;i++){
        if (((altitudeBuffer[i][altitudeIndex] - lastAcceptedAltitudes[i])/(millis() - lastAcceptedTimes[i])) > MAX_VELOCITY){
            markFailure(i);
        }
    }
}


/*
 * Function: consensousCheck
 * -------------------
 * This function returns a higher precision altitude value
 * based on the US 1976 Standard Atmosphere.
 */
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

/*********************************  HELPERS  **********************************/
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

/*
 * Function: getNumRejections
 * -------------------
 * This function returns the numer of rejections
 * a specific sensor has encountered.s
 */
uint32_t Filters::getNumRejections(uint8_t sensor) {
	return rejectedSensors[sensor - 1];
}

/*
 * Function: markFailure
 * -------------------
 * This function marks a specific
 * sensor failure.
 */
void Filters::markFailure(uint8_t sensor){
    if(enabledSensors[sensor]) rejectedSensors[sensor]++;
	enabledSensors[sensor] = false;
    altitudeErrors[sensor][altitudeIndex] = true;
}
