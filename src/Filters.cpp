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
  // char filename[] = "FILTER00.TXT";
  // for (uint8_t i = 0; i < 100; i++) {
  //   filename[6] = i / 10 + '0';
  //   filename[7] = i % 10 + '0';
  //   if (!SD.exists(filename)) {
  //     debugFile = SD.open(filename, FILE_WRITE);
  //     break;
  //   }
  // }
  // if (!debugFile) {
  //   Serial.println ("ERROR: COULD NOT CREATE DEBUG FILE");
  // }
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
float Filters::getTemp(float RAW_TEMP_1, float RAW_TEMP_2, float RAW_TEMP_3, float RAW_TEMP_4) {
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
void Filters::storeData(uint32_t time_stamp, float RAW_PRESSURE_1, float RAW_PRESSURE_2, float RAW_PRESSURE_3, float RAW_PRESSURE_4, float pressureBaselineArg){
  pressureBaseline = pressureBaselineArg;
  altitudeIndex = (altitudeIndex + 1) % ALTITUDE_BUFFER_SIZE;

  if(altitudeIndex == 0) firstBUFFER = false;

  // debugFile.flush();
  // debugFile.print("\n");
  // debugFile.print("time_stamp "); debugFile.print(time_stamp);
  // debugFile.print("alt index ");debugFile.print(altitudeIndex);
  // debugFile.print("\n");

  for (size_t i = 0; i < 4; i++){
    if(!altitudeErrors[i][altitudeIndex]){
      // debugFile.print("sensor  "); debugFile.print(i);
      // debugFile.print(" removing ");
      // debugFile.print(" alt ") ; debugFile.print(altitudeBuffer[i][altitudeIndex]);
      // debugFile.print(" time ") ; debugFile.print(sampleTimeSeconds[altitudeIndex]) ;
      // debugFile.print("\n");
      sumX[i] -= sampleTimeSeconds[altitudeIndex];
      sumY[i] -= altitudeBuffer[i][altitudeIndex];
      sumXY[i] -= sampleTimeSeconds[altitudeIndex] * altitudeBuffer[i][altitudeIndex];
      sumX2[i] -= pow(sampleTimeSeconds[altitudeIndex],2);
      sampleCount[i]--;
    }
  }

  sampleTimeSeconds[altitudeIndex] = (float)time_stamp/1000;

  // debugFile.print(" sampleTimeSeconds "); debugFile.print(sampleTimeSeconds[altitudeIndex]);
  // debugFile.print("\n");

  pressures[0] = RAW_PRESSURE_1;
  pressures[1] = RAW_PRESSURE_2;
  pressures[2] = RAW_PRESSURE_3;
  pressures[3] = RAW_PRESSURE_4;

  // debugFile.print("pressures "); debugFile.print(RAW_PRESSURE_1);
  // debugFile.print(" "); debugFile.print(RAW_PRESSURE_2);
  // debugFile.print(" ") ; debugFile.print(RAW_PRESSURE_3);
  // debugFile.print(" ") ; debugFile.print(RAW_PRESSURE_4);
  // debugFile.print("\n");

  for(int i = 0; i<4;i++) altitudeErrors[i][altitudeIndex] = !enabledSensors[i];
  errorCheckAltitudes();

  for (size_t i = 0; i < 4; i++){
    // debugFile.print("sensor "); debugFile.print(i);
    // debugFile.print(" last Alt ") ; debugFile.print(lastAcceptedAltitudes[i]);
    // debugFile.print(" last Time ") ; debugFile.print(lastAcceptedTimes[i]);
    if(!altitudeErrors[i][altitudeIndex]){
      // debugFile.print(" adding  ");
      // debugFile.print(" alt ") ; debugFile.print(altitudeBuffer[i][altitudeIndex]);
      // debugFile.print(" time ") ; debugFile.print(sampleTimeSeconds[altitudeIndex]) ;
      // debugFile.print("\n");

      sumX[i] += sampleTimeSeconds[altitudeIndex];
      sumY[i] += altitudeBuffer[i][altitudeIndex];
      sumXY[i] += sampleTimeSeconds[altitudeIndex] * altitudeBuffer[i][altitudeIndex];
      sumX2[i] += pow(sampleTimeSeconds[altitudeIndex],2);
      sampleCount[i]++;

      // debugFile.print("SumX  "); debugFile.print(sumY[i]);
      // debugFile.print(" SumY  "); debugFile.print(sumX[i]);
      // debugFile.print(" SumXY  "); debugFile.print(sumXY[i]);
      // debugFile.print(" SumX2  "); debugFile.print(sumX2[i]);

    }
    // debugFile.print(" sampleCount "); debugFile.print(sampleCount[i]);
    // debugFile.print("\n");
  }
}

/*
* Function: getAverageCurrentSystem
* -------------------
* This function returns the average subsytem current over the last window.
*/
float   Filters::getAvgCurrentSystem(float current) {
  if(current > currentSystemMax) currentSystemMax = current;
  if(current < currentSystemMin) currentSystemMin = current;
  currentSystemTotal += current;
  currentSystemCount++;
  return currentSystemTotal / currentSystemCount;
}
/*
* Function: getAverageCurrentRB
* -------------------
* This function returns the average subsytem current over the last window.
*/
float Filters::getAvgCurrentRB(float current) {
  if(current > currentRBMax) currentRBMax = current;
  currentRBTotal += current;
  currentRBCount++;
  return currentRBTotal / currentRBCount;
}

/*
* Function: getAverageCurrentMotorValve
* -------------------
* This function returns the average subsytem current over the last window.
*/
float Filters::getAvgCurrentMotorValve(float current,bool on) {
  if(on) {
    if(current > currentMotorValveMax) currentMotorValveMax = current;
    currentMotorValveTotal += current;
    currentMotorValveCount++;
  }
  return ((currentMotorValveCount != 0) ? (currentMotorValveTotal / currentMotorValveCount): 0);
}

/*
* Function: getAverageCurrentMotorBallast
* -------------------
* This function returns the average subsytem current over the last window.
*/
float Filters::getAvgCurrentMotorBallast(float current,bool on) {
  if(on) {
    if(current > currentMotorBallastMax) currentMotorBallastMax = current;
    currentMotorBallastTotal += current;
    currentMotorBallastCount++;
  }
  return ((currentMotorBallastCount != 0) ? (currentMotorBallastTotal / currentMotorBallastCount): 0);
}

/*
* Function: getAverageCurrentPayload
* -------------------
* This function returns the average subsytem current over the last window.
*/
float Filters::getAvgCurrentPayload(float current) {
  if(current > currentPayloadMax) currentPayloadMax = current;
  currentPayloadTotal += current;
  currentPayloadCount++;
  return currentPayloadTotal / currentPayloadCount;
}

/*
* Function: getMaxCurrentSystem
* -------------------
* This function returns the maximum subsytem current over the last window.
*/
float Filters::getMaxCurrentSystem() {
  return currentSystemMax;
}

/*
* Function: getMinCurrentSystem
* -------------------
* This function returns the minimum subsytem current over the last window.
*/
float Filters::getMinCurrentSystem() {
  return currentSystemMin;
}

/*
* Function: getMaxCurrentRB
* -------------------
* This function returns the maximum subsytem current over the last window.
*/
float Filters::getMaxCurrentRB() {
  return currentRBMax;
}

/*
* Function: getMaxCurrentMotorValve
* -------------------
* This function returns the maximum subsytem current over the last window.
*/
float Filters::getMaxCurrentMotorValve() {
  return currentMotorValveMax;
}

/*
* Function: getMaxCurrentMotorBallast
* -------------------
* This function returns the maximum subsytem current over the last window.
*/
float Filters::getMaxCurrentMotorBallast() {
  return currentMotorBallastMax;
}

/*
* Function: getMaxCurrentPayload
* -------------------
* This function returns the maximum subsytem current over the last window.
*/
float Filters::getMaxCurrentPayload() {
  return currentPayloadMax;
}

/*
* Function: clearCurrentValues
* -------------------
* This function clears the current values for the system variables.
*/
void Filters::clearCurrentValues() {
  currentSystemTotal = 0;
  currentSystemMax = 0;
  currentSystemMin = 10000;
  currentSystemCount = 0;
  currentRBTotal = 0;
  currentRBMax = 0;
  currentRBCount = 0;
  currentMotorValveTotal = 0;
  currentMotorValveMax = 0;
  currentMotorValveCount = 0;
  currentMotorBallastTotal = 0;
  currentMotorBallastMax = 0;
  currentMotorBallastCount = 0;
  currentPayloadTotal = 0;
  currentPayloadMax = 0;
  currentPayloadCount = 0;
  for(size_t i = 0; i < 4; i ++) rejectedSensors[i] = 0;
}
/***************************  GET FUNCTIONS  **********************************/

/*
* Function: getAltitude
* -------------------
* This function returns an error checked and smoothed
* altitude value
*/
double Filters::getAltitude(){
  float sumOfAltitudes = 0;
  int acceptedStreams = 0;
  for(int i = 0; i<4;i++){
    if((sampleCount[i] >= MINIMUM_ALTITUDE_POINTS) && (enabledSensors[i] == true)){
      sumOfAltitudes += sumY[i]/sampleCount[i];
      acceptedStreams++;
    }
  }


  if(acceptedStreams == 0){
    sumOfAltitudes = 0;
    acceptedStreams = 0;
    for(int i = 0; i<4;i++){
      if((sampleCount[i] != 0 ) && (enabledSensors[i] == true)){
        sumOfAltitudes += sumY[i]/sampleCount[i];
        acceptedStreams++;
      }
    }
      if(acceptedStreams == 0){
        sumOfAltitudes = 0;
        acceptedStreams = 0;
        for(int i = 0; i<4;i++){
          if(enabledSensors[i] == true){
            sumOfAltitudes += altitudeBuffer[i][altitudeIndex];
            acceptedStreams++;
          }
        }
      }
  }

  // debugFile.print("meanAltitude"); debugFile.print((sumOfAltitudes/acceptedStreams));
  // debugFile.print(" altitude acceptedStreams "); debugFile.print((acceptedStreams)); debugFile.print("\n");

  return sumOfAltitudes / acceptedStreams;
}

/*
* Function: getAscentRate
* -------------------
* This function returns filtered and smoothed
* ascent rate value
*/
double Filters::getAscentRate() {
  double sumOfAscentRates= 0;
  double meanAscentRates[4] = {0};
  int acceptedStreams = 0;

  for(int i = 0; i < 4; i++) {
    meanAscentRates[i] = (sumXY[i] - sumX[i]*sumY[i]/sampleCount[i] )/(sumX2[i] - sumX[i]*sumX[i]/sampleCount[i]);
    if((sampleCount[i] >= MINIMUM_ASCENT_RATE_POINTS) && (enabledSensors[i] == true)){
      sumOfAscentRates+= meanAscentRates[i];
      acceptedStreams++;
    }
  }

  if(acceptedStreams == 0){
    sumOfAscentRates= 0;
    acceptedStreams = 0;
    for(int i = 0; i<4;i++){
      if((!isnan(meanAscentRates[i])) && (enabledSensors[i] == true)){
        sumOfAscentRates+= meanAscentRates[i];
        acceptedStreams++;
      }
    }

    if(acceptedStreams == 0){
      sumOfAscentRates= 0;
      acceptedStreams = 0;
      for(int i = 0; i<4;i++){
        if(enabledSensors[i] == true){
          sumOfAscentRates+= (altitudeBuffer[i][altitudeIndex] - altitudeBuffer[i][(altitudeIndex + 1)%ALTITUDE_BUFFER_SIZE])/
                            (sampleTimeSeconds[altitudeIndex] - sampleTimeSeconds[(altitudeIndex + 1)%ALTITUDE_BUFFER_SIZE]);
          acceptedStreams++;
        }
      }
    }

  }

  // debugFile.print("meanAscentRate "); debugFile.print((sumOfAscentRates/acceptedStreams));
  // debugFile.print(" AscentRate acceptedStreams "); debugFile.print((acceptedStreams)); debugFile.print("\n");

  return sumOfAscentRates/ acceptedStreams;
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
  if(numSensors == 0){
    numSensors = 4;
    for(int i = 0; i<4;i++) press += pressures[i];
  }
  else {
    for(int i = 0; i<4;i++) if (!altitudeErrors[i][altitudeIndex]) press += pressures[i];
  }

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
  //disable checks during the first loop
  if(!firstBUFFER){
    // debugFile.print("altitudes 1: "); debugFile.print(altitudeBuffer[0][altitudeIndex]);
    // debugFile.print(" 2: "); debugFile.print(altitudeBuffer[1][altitudeIndex]);
    // debugFile.print(" 3: "); debugFile.print(altitudeBuffer[2][altitudeIndex]);
    // debugFile.print(" 4: "); debugFile.print(altitudeBuffer[3][altitudeIndex]);
    // debugFile.print("\n");
    // debugFile.print("Errors org1: "); debugFile.print(altitudeErrors[0][altitudeIndex]);
    // debugFile.print(" 2: "); debugFile.print(altitudeErrors[1][altitudeIndex]);
    // debugFile.print(" 3: "); debugFile.print(altitudeErrors[2][altitudeIndex]);
    // debugFile.print(" 4: "); debugFile.print(altitudeErrors[3][altitudeIndex]);
    // debugFile.print("\n");
    consensousCheck();
    // debugFile.print("Errors conc1: "); debugFile.print(altitudeErrors[0][altitudeIndex]);
    // debugFile.print(" 2: "); debugFile.print(altitudeErrors[1][altitudeIndex]);
    // debugFile.print(" 3: "); debugFile.print(altitudeErrors[2][altitudeIndex]);
    // debugFile.print(" 4: "); debugFile.print(altitudeErrors[3][altitudeIndex]);
    // debugFile.print("\n");
    velocityCheck();
    // debugFile.print("Errors vel1: "); debugFile.print(altitudeErrors[0][altitudeIndex]);
    // debugFile.print(" 2: "); debugFile.print(altitudeErrors[1][altitudeIndex]);
    // debugFile.print(" 3: "); debugFile.print(altitudeErrors[2][altitudeIndex]);
    // debugFile.print(" 4: "); debugFile.print(altitudeErrors[3][altitudeIndex]);
    // debugFile.print("\n");
  }
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
      if (((fabs(altitudeBuffer[i][altitudeIndex] - lastAcceptedAltitudes[i])-6*ALTITUDE_STANDARD_DEV)/(sampleTimeSeconds[altitudeIndex] - lastAcceptedTimes[i])) > MAX_VELOCITY){
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
double Filters::calculateAltitude(float pressure) {
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
