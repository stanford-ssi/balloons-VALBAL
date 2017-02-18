/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu

  File: Avionics.cpp
  --------------------------
  Implimentation of Avionics.h
*/

#include "Avionics.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the avionics flight controller.
 */
void Avionics::init() {
  Serial.begin(9600);
  PCB.init();
  printHeader();
  if(!SD.begin(SD_CS)) PCB.faultLED();
  logHeader();
  if(!sensors.init())     logAlert("unable to initialize Sensors", true);
  if(!computer.init())    logAlert("unable to initialize Flight Controller", true);
  if(!gpsModule.init())   logAlert("unable to initialize GPS", true);
  if(!RBModule.init())    logAlert("unable to initialize RockBlock", true);
  data.SETUP_STATE = false;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateData
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateState() {
  if(!readData()) logAlert("unable to read Data", true);
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently calculates the current state.
 */
void Avionics::evaluateState() {
  if(!calcState())  logAlert("unable to calculate state", true);
}

/*
 * Function: actuateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::actuateState() {
  if(!debugState()) logAlert("unable to debug state", true);
  if(!runHeaters()) logAlert("unable to run heaters", true);
  if(!runValve()) logAlert("unable to run valve", true);
  if(!runBalast()) logAlert("unable to run balast", true);
  if(!runCutdown()) logAlert("unable to run cutdown", true);
}

/*
 * Function: logState
 * -------------------
 * This function logs the current data frame.
 */
void Avionics::logState() {
  if(compressData() < 0) logAlert("unable to compress Data", true);
  if(!logData())         logAlert("unable to log Data", true);
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
  if((millis() - data.COMMS_LAST) < COMMS_RATE) return;
  if(!sendSATCOMS()) logAlert("unable to communicate over RB", true);
  data.COMMS_LAST = millis();
}

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  gpsModule.smartDelay(LOOP_RATE);
}

/*
 * Function: finishedSetup
 * -------------------
 * This function returns true if the avionics has completed setup.
 */
bool Avionics::finishedSetup() {
  return !data.SETUP_STATE;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
bool Avionics::readData() {
  data.TIME            = sensors.getTime();
  data.LOOP_GOOD_STATE = !data.LOOP_GOOD_STATE;
  data.LOOP_RATE       = millis() - data.LOOP_START;
  data.LOOP_START      = millis();
  data.ALTITUDE_LAST   = data.ALTITUDE_BMP;
  data.VOLTAGE         = sensors.getVoltage();
  data.CURRENT         = sensors.getCurrent();
  data.TEMP            = sensors.getTemp();
  data.PRESS_BMP       = sensors.getPressure();
  data.ALTITUDE_BMP    = sensors.getAltitude();
  data.ASCENT_RATE     = sensors.getAscentRate();
  data.LAT_GPS         = gpsModule.getLatitude();
  data.LONG_GPS        = gpsModule.getLongitude();
  data.ALTITUDE_GPS    = gpsModule.getAltitude();
  data.HEADING_GPS     = gpsModule.getCourse();
  data.SPEED_GPS       = gpsModule.getSpeed();
  data.NUM_SATS_GPS    = gpsModule.getSats();
  return true;
}

/*
 * Function: calcState
 * -------------------
 * This function sets the appropriate values and flags based on the current data frame.
 */
bool Avionics::calcState() {
  calcVitals();
  calcDebug();
  calcCutdown();
  data.valveIncentive = computer.getBalastIncentive();
  data.ballastIncentive = computer.getValveIncentive();
  return true;
}

/*
 * Function: debugState
 * -------------------
 * This function provides debuging information.
 */
bool Avionics::debugState() {
  if(data.DEBUG_STATE) {
    printState();
  }
  return true;
}

/*
 * Function: runHeaters
 * -------------------
 * This function thermaly regulates the avionics.
 */
bool Avionics::runHeaters() {
  PCB.heater(data.TEMP);
  return true;
}

/*
 * Function: runValve
 * -------------------
 * This function actuates the valve based on the calcualted incentive.
 */
bool Avionics::runValve() {
  if(data.FORCE_VALVE) PCB.valve(true);
  else if(data.valveIncentive >= 1) PCB.valve(false);
  return true;
}

/*
 * Function: runBalast
 * -------------------
 * This function actuates the valve based on the calcualted incentive.
 */
bool Avionics::runBalast() {
  if(data.FORCE_BALAST) PCB.balast(true);
  else if(data.ballastIncentive >= 1) PCB.balast(false);
  return true;
}

/*
 * Function: runCutdown
 * -------------------
 * This function cuts down the payload if nessisary.
 */
bool Avionics::runCutdown() {
  if(data.CUTDOWN_STATE) return true;
  if(data.SHOULD_CUTDOWN) {
    PCB.cutDown(true);
    gpsModule.smartDelay(CUTDOWN_TIME);
    PCB.cutDown(false);
    data.CUTDOWN_STATE = true;
    logAlert("completed cutdown", false);
  }
  return true;
}

/*
 * Function: sendSATCOMS
 * -------------------
 * This function sends the current data frame over the ROCKBLOCK IO.
 */
bool Avionics::sendSATCOMS() {
  logAlert("sending Rockblock message", false);
  data.RB_SENT_COMMS++;
  int16_t ret = RBModule.writeRead(COMMS_BUFFER, data.COMMS_LENGTH);
  if(ret < 0) {
    data.RB_GOOD_STATE  = false;
    return false;
  }
  data.RB_GOOD_STATE  = true;
  if(ret > 0) parseCommand(ret);
  return true;
}

/*
 * Function: parseCommand
 * -------------------
 * This function parses the command received from the RockBLOCK.
 */
void Avionics::parseCommand(int16_t len) {
  if(strncmp(COMMS_BUFFER, CUTDOWN_COMAND, len)) {
    data.SHOULD_CUTDOWN = true;
  }
}

/*
 * Function: calcVitals
 * -------------------
 * This function calculates if the current state is within bounds.
 */
void Avionics::calcVitals() {
  data.BAT_GOOD_STATE    = (data.VOLTAGE >= 3.63);
  data.CURR_GOOD_STATE   = (data.CURRENT > -5.0 && data.CURRENT <= 500.0);
  data.PRES_GOOD_STATE   = (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200);
  data.TEMP_GOOD_STATE   = (data.TEMP > 15 && data.TEMP < 50);
  data.GPS_GOOD_STATE    = (data.LAT_GPS != 1000.0 && data.LAT_GPS != 0.0 && data.LONG_GPS != 1000.0 && data.LONG_GPS != 0.0);
}

/*
 * Function: calcDebug
 * -------------------
 * This function calculates if the avionics is in debug mode.
 */
void Avionics::calcDebug() {
  if(data.DEBUG_STATE   && (data.ALTITUDE_LAST >= DEBUG_ALT) && (data.ALTITUDE_BMP >= DEBUG_ALT)) {
    data.DEBUG_STATE = false;
  }
}

/*
 * Function: calcCutdown
 * -------------------
 * This function calculates if the avionics should cutdown.
 */
void Avionics::calcCutdown() {
  if(CUTDOWN_GPS_ENABLE && data.GPS_GOOD_STATE &&
    (((data.LAT_GPS < GPS_FENCE_LAT_MIN) || (data.LAT_GPS > GPS_FENCE_LAT_MAX)) ||
    ((data.LONG_GPS < GPS_FENCE_LON_MIN) || (data.LONG_GPS > GPS_FENCE_LON_MAX)))
  ) data.SHOULD_CUTDOWN  = true;

  if(CUTDOWN_ALT_ENABLE && !data.CUTDOWN_STATE &&
    (data.ALTITUDE_LAST >= CUTDOWN_ALT) &&
    (data.ALTITUDE_BMP  >= CUTDOWN_ALT)
  ) data.SHOULD_CUTDOWN  = true;
}

/*
 * Function: printHeader
 * -------------------
 * This function prints the CSV header.
 */
void Avionics::printHeader() {
  if(!Serial) PCB.faultLED();
  Serial.print("Stanford Student Space Initiative Balloons Launch ");
  Serial.print(MISSION_NUMBER);
  Serial.print('\n');
  Serial.print(CSV_DATA_HEADER);
  Serial.print('\n');
}

/*
 * Function: logHeader
 * -------------------
 * This function logs the CSV header.
 */
void Avionics::logHeader() {
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(!dataFile) PCB.faultLED();
  dataFile.print("Stanford Student Space Initiative Balloons Launch ");
  dataFile.print(MISSION_NUMBER);
  dataFile.print('\n');
  dataFile.print(CSV_DATA_HEADER);
  dataFile.print('\n');
  dataFile.flush();
  dataFile.close();
}

/*
 * Function: logAlert
 * -------------------
 * This function logs important information whenever a specific event occurs.
 */
void Avionics::logAlert(const char* debug, bool fatal) {
  if(fatal) PCB.faultLED();
  dataFile = SD.open("log.txt", FILE_WRITE);
  if(dataFile) {
    dataFile.print(data.TIME);
    dataFile.print(',');
    if(fatal) dataFile.print("FATAL ERROR!!!!!!!!!!: ");
    else dataFile.print("Alert: ");
    dataFile.print(debug);
    dataFile.print("...\n");
    dataFile.flush();
    dataFile.close();
  }
  if(data.DEBUG_STATE) {
    Serial.print(data.TIME);
    Serial.print(',');
    if(fatal) Serial.print("FATAL ERROR!!!!!!!!!!: ");
    else Serial.print("Alert: ");
    Serial.print(debug);
    Serial.print("...\n");
  }
}

/*
 * Function: printState
 * -------------------
 * This function prints the current avionics state.
 */
void Avionics::printState() {
  Serial.print(data.TIME);
  Serial.print(',');
  Serial.print(data.LOOP_RATE);
  Serial.print(',');
  Serial.print(data.VOLTAGE);
  Serial.print(',');
  Serial.print(data.CURRENT);
  Serial.print(',');
  Serial.print(data.ALTITUDE_BMP);
  Serial.print(',');
  Serial.print(data.ASCENT_RATE);
  Serial.print(',');
  Serial.print(data.TEMP);
  Serial.print(',');
  Serial.print(data.LAT_GPS, 4);
  Serial.print(',');
  Serial.print(data.LONG_GPS, 4);
  Serial.print(',');
  Serial.print(data.SPEED_GPS);
  Serial.print(',');
  Serial.print(data.HEADING_GPS);
  Serial.print(',');
  Serial.print(data.ALTITUDE_GPS);
  Serial.print(',');
  Serial.print(data.PRESS_BMP);
  Serial.print(',');
  Serial.print(data.NUM_SATS_GPS);
  Serial.print(',');
  Serial.print(data.RB_SENT_COMMS);
  Serial.print(',');
  Serial.print(data.CUTDOWN_STATE);
  Serial.print('\n');
}

/*
 * Function: logData
 * -------------------
 * This function logs the current data frame.
 */
bool Avionics::logData() {
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(!dataFile) return false;
  dataFile.print(data.TIME);
  dataFile.print(',');
  dataFile.print(data.LOOP_RATE);
  dataFile.print(',');
  dataFile.print(data.VOLTAGE);
  dataFile.print(',');
  dataFile.print(data.CURRENT);
  dataFile.print(',');
  dataFile.print(data.ALTITUDE_BMP);
  dataFile.print(',');
  dataFile.print(data.ASCENT_RATE);
  dataFile.print(',');
  dataFile.print(data.TEMP);
  dataFile.print(',');
  dataFile.print(data.LAT_GPS, 4);
  dataFile.print(',');
  dataFile.print(data.LONG_GPS, 4);
  dataFile.print(',');
  dataFile.print(data.SPEED_GPS);
  dataFile.print(',');
  dataFile.print(data.HEADING_GPS);
  dataFile.print(',');
  dataFile.print(data.ALTITUDE_GPS);
  dataFile.print(',');
  dataFile.print(data.PRESS_BMP);
  dataFile.print(',');
  dataFile.print(data.NUM_SATS_GPS);
  dataFile.print(',');
  dataFile.print(data.RB_SENT_COMMS);
  dataFile.print(',');
  dataFile.print(data.CUTDOWN_STATE);
  dataFile.print('\n');
  dataFile.flush();
  dataFile.close();
  return true;
}

/*
 * Function: compressData
 * -------------------
 * This function compresses the data frame into a bit stream.
 */
int16_t Avionics::compressData() {
  int16_t length = 0;
  size_t varSize = 0;

  varSize = sizeof(data.TIME);
  memcpy(COMMS_BUFFER + length, &data.TIME, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.LOOP_RATE);
  memcpy(COMMS_BUFFER + length, &data.LOOP_RATE, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.VOLTAGE);
  memcpy(COMMS_BUFFER + length, &data.VOLTAGE, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.CURRENT);
  memcpy(COMMS_BUFFER + length, &data.CURRENT, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.ALTITUDE_BMP);
  memcpy(COMMS_BUFFER + length, &data.ALTITUDE_BMP, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.ASCENT_RATE);
  memcpy(COMMS_BUFFER + length, &data.ASCENT_RATE, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.TEMP);
  memcpy(COMMS_BUFFER + length, &data.TEMP, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.LAT_GPS);
  memcpy(COMMS_BUFFER + length, &data.LAT_GPS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.LONG_GPS);
  memcpy(COMMS_BUFFER + length, &data.LONG_GPS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.SPEED_GPS);
  memcpy(COMMS_BUFFER + length, &data.SPEED_GPS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.HEADING_GPS);
  memcpy(COMMS_BUFFER + length, &data.HEADING_GPS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.ALTITUDE_GPS);
  memcpy(COMMS_BUFFER + length, &data.ALTITUDE_GPS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.PRESS_BMP);
  memcpy(COMMS_BUFFER + length, &data.PRESS_BMP, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.NUM_SATS_GPS);
  memcpy(COMMS_BUFFER + length, &data.NUM_SATS_GPS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.RB_SENT_COMMS);
  memcpy(COMMS_BUFFER + length, &data.RB_SENT_COMMS, varSize);
  length += varSize;
  COMMS_BUFFER[length] = ','; length++;

  varSize = sizeof(data.CUTDOWN_STATE);
  memcpy(COMMS_BUFFER + length, &data.CUTDOWN_STATE, varSize);
  length += varSize;
  data.COMMS_LENGTH = length;
  return length;
}
