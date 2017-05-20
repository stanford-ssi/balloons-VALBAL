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
double Filters::getPressure(double RAW_PRESSURE_1, double RAW_PRESSURE_2, double RAW_PRESSURE_3, double RAW_PRESSURE_4,double pressureBaselineArg) {
    pressureBaseline = pressureBaselineArg;
    filtered = false;
    altitudeIndex = (altitudeIndex + 1) % ALTITUDE_BUFFER_SIZE;

    Serial.print("ALT INDEX ");
    Serial.print(altitudeIndex);
    Serial.print("\n\r");

    for(int i = 0; i<4;i++) altitudeErrors[i][altitudeIndex] = false;

    pressures[0] = RAW_PRESSURE_1;
    pressures[1] = RAW_PRESSURE_2;
    pressures[2] = RAW_PRESSURE_3;
    pressures[3] = RAW_PRESSURE_4;


    for(int i = 0; i<4;i++) if(!((MIN_PRESURE < pressures[i]) && (pressures[i] < MAX_PRESURE))) markFailure(i);

    int numSensors = 0;
    for (size_t i = 0; i < 4; i++) if (!altitudeErrors[i][altitudeIndex]) numSensors++;
    // Calculate mean of sensors which passed

    double press = 0;
    for(int i = 0; i<4;i++) if (!altitudeErrors[i][altitudeIndex]) press += pressures[i];

    return press / numSensors;
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
    return currentMotorsTotal / currentMotorsCount;
}

/*
* Function: getAverageCurrentPayload
* -------------------
* This function returns the average subsytem current over the last window.
*/
double   Filters::getAverageCurrentPayload(double current) {
    currentPayloadTotal += current;
    currentPayloadCount++;
    return currentPayloadTotal / currentPayloadCount;
}

/*
* Function: clearAverages
* -------------------
* This function clears the current average values for the system variables.
*/
void   Filters::clearAverages() {
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
* Function: getAscentRate
* -------------------
* This function returns filtered and smoothed
* ascent rate value
*/
double Filters::getAscentRate() {
    if(!filtered) Serial.print("ERROR!: getAltitude needs to be called before getAscentRate");

    float meanAscentRate = 0;
    int acceptedStreams = 0;

    for(int i = 0; i < 4; i++){
        double numerator = 0;
        double denominator = 0;

        double meanX = 0; //(time)

        for(int t = 0; t < ALTITUDE_BUFFER_SIZE; t++){

            float time_seconds = (float)sampleTime[t]/1000;
            // int time_seconds = (double)LOOP_INTERVAL*((t - 1 - altitudeIndex + ALTITUDE_BUFFER_SIZE) % ALTITUDE_BUFFER_SIZE)/1000;
            if(!altitudeErrors[i][t]){
                meanX += time_seconds;
            }
        }
        meanX /= numberOfAcceptedSamples[i];
        for(int t = 0; t < ALTITUDE_BUFFER_SIZE; t++){
            float time_seconds = (float)sampleTime[t]/1000;
            // int time_seconds = (double)LOOP_INTERVAL*((t - 1 - altitudeIndex + ALTITUDE_BUFFER_SIZE) % ALTITUDE_BUFFER_SIZE)/1000;
            if(!altitudeErrors[i][t]){
                numerator += (time_seconds - meanX) * (altitudeBuffer[i][t] - meanAltitudes[i]);
                denominator += pow((time_seconds - meanX),2);
            }
        }

        Serial.print("ascent rate ");
        Serial.print(i);

        Serial.print(" n ");
        Serial.print(numerator);
        Serial.print(" d ");
        Serial.print(denominator);
        Serial.print("\n\r");

        meanAscentRates[i] = numerator/denominator;
        if((numberOfAcceptedSamples[i] >= MINIMUM_ASCENT_RATE_POINTS) && (enabledSensors[i] == true)){
            meanAscentRate += meanAscentRates[i];
            acceptedStreams++;
        }
    }

    if(acceptedStreams == 0) return (meanAscentRates[0] + meanAscentRates[1] + meanAscentRates[2] + meanAscentRates[3])/4;


    // Serial.print("ascent rate");
    // Serial.print(meanAscentRate/acceptedStreams);
    // Serial.print("\n\r");
    return meanAscentRate/acceptedStreams;
}

/*
* Function: getAltitude
* -------------------
* This function returns an error checked and smoothed
* altitude value
*/
double Filters::getAltitude(uint32_t sample_time) {

    sampleTime[altitudeIndex] = sample_time;
    // if(!filtered) Serial.print("error chceking in altitude\n\r");
    if(!filtered) errorCheckAltitudes();

    // Serial.print("buffer out at index prev");
    // Serial.print(altitudeBuffer[0][altitudeIndex]);
    // Serial.print("\n\r");

    // for(int t = 0; t<ALTITUDE_BUFFER_SIZE;t++){
    //     Serial.print("altitude ");
    //     Serial.print(t);
    //     Serial.print(" ");
    //     Serial.print(altitudeErrors[0][t]);
    //     Serial.print(" ");
    //     Serial.print(altitudeBuffer[0][t]);
    //
    //     Serial.print(" | ");
    //     Serial.print(altitudeErrors[1][t]);
    //     Serial.print(" ");
    //     Serial.print(altitudeBuffer[1][t]);
    //
    //     Serial.print(" | ");
    //     Serial.print(altitudeErrors[2][t]);
    //     Serial.print(" ");
    //     Serial.print(altitudeBuffer[2][t]);
    //
    //     Serial.print(" | ");
    //     Serial.print(altitudeErrors[3][t]);
    //     Serial.print(" ");
    //     Serial.print(altitudeBuffer[3][t]);
    //     Serial.print("\n\r");
    //
    // }

    float meanAltitude = 0;
    int acceptedStreams = 0;

    for(int i = 0; i<4;i++){
        float altitudesSum =0;
        numberOfAcceptedSamples[i] = 0;

        for(int t = 0; t<ALTITUDE_BUFFER_SIZE;t++){
            if(!altitudeErrors[i][t]){
                altitudesSum += altitudeBuffer[i][t];
                numberOfAcceptedSamples[i]++;

            }
        }


        meanAltitudes[i] = altitudesSum / numberOfAcceptedSamples[i];

        Serial.print("mean alt");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(meanAltitudes[i]);
        Serial.print(" with acepted smaples ");
        Serial.print(numberOfAcceptedSamples[i]);
        Serial.print("\n\r");
        if((numberOfAcceptedSamples[i] >= MINIMUM_ALTITUDE_POINTS) && (enabledSensors[i] == true)){
            meanAltitude += meanAltitudes[i];
            acceptedStreams++;
        }
    }


    if(acceptedStreams == 0) return (meanAltitudes[0] + meanAltitudes[1] + meanAltitudes[2] + meanAltitudes[3])/4;

    else return meanAltitude / acceptedStreams;
}


/********************************  CHECKERS  **********************************/
/*
* Function: errorCheckAltitudes
* -------------------
* Does the altitude calculation and error checking
*/
void Filters::errorCheckAltitudes() {

    for(int i = 0; i<4;i++) altitudeBuffer[i][altitudeIndex] = calculateAltitude(pressures[i]);


    // Serial.print("currint alt index");
    // Serial.print(altitudeIndex);
    // Serial.print("\n\r");

    consensousCheck();
    velocityCheck();
    findLastAccepted();

    filtered = true;
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
            lastAcceptedTimes[i] = sampleTime[altitudeIndex];
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
        // 1000 comes from times being in miliseconds
        if (fabs(1000* (altitudeBuffer[i][altitudeIndex] - lastAcceptedAltitudes[i])/(sampleTime[altitudeIndex] - lastAcceptedTimes[i])) > MAX_VELOCITY){
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

    // Serial.print("unfilterd alt");
    // Serial.print(calculatedAltitude);
    // Serial.print("\n\r");
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
