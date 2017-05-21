/*
Stanford Student Space Initiative
Balloons | VALBAL | May 2017
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
void Filters::storeData(uint32_t time_stamp, double RAW_PRESSURE_1, double RAW_PRESSURE_2, double RAW_PRESSURE_3, double RAW_PRESSURE_4, double pressureBaselineArg){
    pressureBaseline = pressureBaselineArg;
    altitudeIndex = (altitudeIndex + 1) % ALTITUDE_BUFFER_SIZE;

    for (size_t i = 0; i < 4; i++){
        if(!altitudeErrors[i][altitudeIndex]){
            // Serial.print("senros  "); Serial.print(i);

            // Serial.print("removing index "); Serial.print(altitudeIndex);
            // Serial.print(" alt ") ; Serial.print(altitudeBuffer[i][altitudeIndex]);
            // Serial.print(" time ") ; Serial.print(sampleTimeSeconds[altitudeIndex]) ;
            //   Serial.print("\n\r");
            sumX[i] -= sampleTimeSeconds[altitudeIndex];
            sumY[i] -= altitudeBuffer[i][altitudeIndex];
            sumXY[i] -= sampleTimeSeconds[altitudeIndex] * altitudeBuffer[i][altitudeIndex];
            sumX2[i] -= pow(sampleTimeSeconds[altitudeIndex],2);
            sampleCount[i]--;
         }
    }

    sampleTimeSeconds[altitudeIndex] = (float)time_stamp/1000;
    pressures[0] = RAW_PRESSURE_1;
    pressures[1] = RAW_PRESSURE_2;
    pressures[2] = RAW_PRESSURE_3;
    pressures[3] = RAW_PRESSURE_4;

    for(int i = 0; i<4;i++) altitudeErrors[i][altitudeIndex] = !enabledSensors[i];
    errorCheckAltitudes();

    for (size_t i = 0; i < 4; i++){
        if(!altitudeErrors[i][altitudeIndex]){
            // Serial.print("senros  "); Serial.print(i);
            // Serial.print("adding index "); Serial.print(altitudeIndex);
            // Serial.print(" alt ") ; Serial.print(altitudeBuffer[i][altitudeIndex]);
            // Serial.print(" time ") ; Serial.print(sampleTimeSeconds[altitudeIndex]) ;
            //   Serial.print("\n\r");


            sumX[i] += sampleTimeSeconds[altitudeIndex];
            sumY[i] += altitudeBuffer[i][altitudeIndex];
            sumXY[i] += sampleTimeSeconds[altitudeIndex] * altitudeBuffer[i][altitudeIndex];
            sumX2[i] += pow(sampleTimeSeconds[altitudeIndex],2);
            sampleCount[i]++;

                // Serial.print("totaly  "); Serial.print(sumY[i]);
                // Serial.print("totalx  "); Serial.print(sumX[i]);

            // Serial.print(" coumt "); Serial.print(sampleCount[i]);
            // Serial.print("\n\r");
        }
    }

}

/*
* Function: getAverageCurrentSystem
* -------------------
* This function returns the average subsytem current over the last window.
*/
double   Filters::getAverageCurrentSystem(double current) {
    currentSystemCount++;
    currentSystemTotal += current;
    return currentSystemTotal / currentSystemCount;
}

/*
* Function: getAverageCurrentGPS
* -------------------
* This function returns the average subsytem current over the last window.
*/
double   Filters::getAverageCurrentGPS(double current) {
    currentGPSTotal += current;
    currentGPSCount++;
    return currentGPSTotal / currentGPSCount;
}

/*
* Function: getAverageCurrentRB
* -------------------
* This function returns the average subsytem current over the last window.
*/
double   Filters::getAverageCurrentRB(double current) {
    currentRBTotal += current;
    currentRBCount++;
    return currentRBTotal / currentRBCount;
}

/*
* Function: getAverageCurrentMotors
* -------------------
* This function returns the average subsytem current over the last window.
*/
double   Filters::getAverageCurrentMotors(double current,bool on) {
    if(on) {
        currentMotorsTotal += current;
        currentMotorsCount++;
    }
    if(currentMotorsCount == 0) return 0;
    return currentMotorsTotal / currentMotorsCount;
}

/*
* Function: getAverageCurrentPayload
* -------------------
* This function returns the average subsytem current over the last window.
*/
double Filters::getAverageCurrentPayload(double current) {
    currentPayloadTotal += current;
    currentPayloadCount++;
    return currentPayloadTotal / currentPayloadCount;
}

/*
* Function: clearAverages
* -------------------
* This function clears the current average values for the system variables.
*/
void Filters::clearAverages() {
    currentSystemTotal  = 0;
    currentSystemCount  = 0;
    currentGPSTotal     = 0;
    currentGPSCount     = 0;
    currentRBTotal      = 0;
    currentRBCount      = 0;
    currentMotorsTotal  = 0;
    currentMotorsCount  = 0;
    currentPayloadTotal = 0;
    currentPayloadCount = 0;
}
/***************************  GET FUNCTIONS  **********************************/

/*
* Function: getAltitude
* -------------------
* This function returns an error checked and smoothed
* altitude value
*/
double Filters::getAltitude(){

    float meanAltitude = 0;
    int acceptedStreams = 0;
    for(int i = 0; i<4;i++){
        if((sampleCount[i] >= MINIMUM_ALTITUDE_POINTS) && (enabledSensors[i] == true)){
            meanAltitude += sumY[i]/sampleCount[i];
            acceptedStreams++;
        }
    }
    // Serial.printf("acceptedStreams %d\n", acceptedStreams);

    if(acceptedStreams == 0) return (sumY[0]/sampleCount[0] + sumY[1]/sampleCount[1] + sumY[2]/sampleCount[2] + sumY[3]/sampleCount[3])/4;
    else return meanAltitude / acceptedStreams;
}

/*
* Function: getAscentRate
* -------------------
* This function returns filtered and smoothed
* ascent rate value
*/
double Filters::getAscentRate() {

    double meanAscentRate = 0;
    double meanAscentRates[4] = {0};
    int acceptedStreams = 0;

    for(int i = 0; i < 4; i++) {
        meanAscentRates[i] = (sumXY[i] - sumX[i]*sumY[i]/sampleCount[i] )/(sumX2[i] - sumX[i]*sumX[i]/sampleCount[i]);
        if((sampleCount[i] >= MINIMUM_ASCENT_RATE_POINTS) && (enabledSensors[i] == true)){
            meanAscentRate += meanAscentRates[i];
            acceptedStreams++;
        }
    }

    if(acceptedStreams == 0) return (meanAscentRates[0] + meanAscentRates[1] + meanAscentRates[2] + meanAscentRates[3])/4;

    return meanAscentRate/acceptedStreams;
}


/*
* Function: getPressure
* -------------------
* This function returns an averaged pressure
*/
double Filters::getPressure() {

    int numSensors = 0;
    for (size_t i = 0; i < 4; i++) if (!altitudeErrors[i][altitudeIndex]) numSensors++;
    // Calculate mean of sensors which passed

    double press = 0;
    for(int i = 0; i<4;i++) if (!altitudeErrors[i][altitudeIndex]) press += pressures[i];
    return press / numSensors;
}

/********************************  CHECKERS  **********************************/
/*
* Function: errorCheckAltitudes
* -------------------
* Does the altitude calculation and error checking
*/
void Filters::errorCheckAltitudes() {

    for(int i = 0; i<4;i++){
        if(!((MIN_PRESURE < pressures[i]) && (pressures[i] < MAX_PRESURE))) markFailure(i);
        altitudeBuffer[i][altitudeIndex] = calculateAltitude(pressures[i]);
    }

    consensousCheck();
    velocityCheck();
    findLastAccepted();
}

/*
* Function: findLastAccepted
* -------------------
* This function updates the last accepted values used by
* velocity check
*/
void Filters::findLastAccepted() {
    for(int i = 0; i<4;i++){
        if (!altitudeErrors[i][altitudeIndex]){
            lastAcceptedAltitudes[i] = altitudeBuffer[i][altitudeIndex];
            lastAcceptedTimes[i] = sampleTimeSeconds[altitudeIndex];
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
        if (fabs((altitudeBuffer[i][altitudeIndex] - lastAcceptedAltitudes[i])/(sampleTimeSeconds[altitudeIndex] - lastAcceptedTimes[i])) > MAX_VELOCITY){
            markFailure(i);
        }
    }
}

/*
* Function: consensousCheck
* -------------------
* Finds the arangemnt with the highest number of
* sensors in agreement with each other
*/
void Filters::consensousCheck(){
    int maxAgreement = 0;
    int maxSensors = 0;
    int minDistance = 0;

    // for each sensor combination
    for(int activeSensors = 1; activeSensors<16; activeSensors++){
        int numberOfSensors = 0;
        int numberOfCorrectSensors = 0;
        float meanAltitude = 0;
        float distance = 0;

        // calculate mean
        for(int sensor = 0; sensor < 4; sensor++){
            if( 1 & (activeSensors>>sensor)){
                numberOfSensors++;
                meanAltitude += altitudeBuffer[sensor][altitudeIndex];
            }
        }
        meanAltitude /= numberOfSensors;

        // count sensors in range
        for(int sensor = 0; sensor < 4; sensor++){
            if(1 & (activeSensors>>sensor)){
                distance += pow(altitudeBuffer[sensor][altitudeIndex] - meanAltitude,2);
                if(fabs(altitudeBuffer[sensor][altitudeIndex] - meanAltitude) < MAX_CONSENSUS_DEVIATION) numberOfCorrectSensors +=1;
            }
        }

        // if arangemnt is better
        if(numberOfCorrectSensors > maxSensors || (numberOfCorrectSensors == maxSensors && distance < minDistance)){
            maxAgreement = activeSensors;
            maxSensors = numberOfSensors;
            minDistance = distance;
        }
    }

    for(int sensor = 0; sensor < 4; sensor++){
        if(!(1 & (maxAgreement>>sensor))){
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
double Filters::calculateAltitude(double pressure) {
    double calculatedAltitude;
    if (pressure > 22632.1) calculatedAltitude = (44330.7 * (1 - pow(pressure / pressureBaseline, 0.190266)));
    else calculatedAltitude =  -6341.73 * log((0.176481 * pressure) / 22632.1);

    return calculatedAltitude;
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
* Function: getIncentiveNoise
* -------------------
* This function calculates the inherent noise of the incentive.
*/
float Filters::getIncentiveNoise(bool IncludeBMP1, bool IncludeBMP2, bool IncludeBMP3, bool IncludeBMP4) {
    float incentiveNoise = 0;
    if(IncludeBMP1 && !enabledSensors[0]) incentiveNoise++;
    if(IncludeBMP2 && !enabledSensors[1]) incentiveNoise++;
    if(IncludeBMP3 && !enabledSensors[2]) incentiveNoise++;
    if(IncludeBMP4 && !enabledSensors[3]) incentiveNoise++;
    return incentiveNoise;
}

/*
* Function: markFailure
* -------------------
* This function marks a specific
* sensor failure.
*/
void Filters::markFailure(uint8_t sensor){
    if(enabledSensors[sensor]) rejectedSensors[sensor]++;
    // enabledSensors[sensor] = false;
    altitudeErrors[sensor][altitudeIndex] = true;
}
