/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
  Davy Ragland | dragland@stanford.edu

  File: Avionics.cpp
  --------------------------
  Implementation of Avionics.h
*/

#include "Avionics.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the avionics flight controller.
 */
void Avionics::init() {
  PCB.init();
  Serial.begin(CONSOLE_BAUD);
  printHeader();
  if(!SD.begin(SD_CS)) PCB.faultLED();
  setupLog();
  logHeader();
  if(!readHistory())                              logAlert("unable to read from EEPROM", true);
  if(!sensors.init())                             logAlert("unable to initialize Sensors", true);
  if(!filter.init())                              logAlert("unable to initialize Filters", true);
  if(!computer.init())                            logAlert("unable to initialize Flight Controller", true);
  if(!gpsModule.init(data.GPS_SHOULD_USE))        logAlert("unable to initialize GPS", true);
  if(!RBModule.init(data.RB_SHOULD_USE))          logAlert("unable to initialize RockBlock", true);
  if(!PCB.startUpHeaters(data.HEATER_SHOULD_USE)) logAlert("unable to initialize Heaters", true);
  data.SETUP_STATE = false;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateState
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateState() {
  if(!readData())    logAlert("unable to read Data", true);
  if(!processData()) logAlert("unable to process Data", true);
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently calculates the current state.
 */
void Avionics::evaluateState() {
  if(!calcVitals())     logAlert("unable to calculate vitals", true);
  if(!calcDebug())      logAlert("unable to calculate debug", true);
  if(!calcIncentives()) logAlert("unable to calculate incentives", true);
  if(!calcCutdown())    logAlert("unable to calculate cutdown", true);
}

/*
 * Function: actuateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::actuateState() {
  if(!runHeaters()) logAlert("unable to run heaters", true);
  if(!runValve())   logAlert("unable to run valve", true);
  if(!runBallast()) logAlert("unable to run ballast", true);
  if(!runCutdown()) logAlert("unable to run cutdown", true);
}

/*
 * Function: logState
 * -------------------
 * This function logs the current data frame.
 */
void Avionics::logState() {
  if(!logData())    logAlert("unable to log Data", true);
  if(!debugState()) logAlert("unable to debug state", true);
  if (data.MINUTES > FILE_RESET_TIME) {
    dataFile.close();
    logFile.close();
    setupLog();
    printHeader();
  }
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
  if(data.DEBUG_STATE && ((millis() - data.COMMS_LAST) < COMMS_DEBUG_RATE)) return;
  if(!data.DEBUG_STATE && ((millis() - data.COMMS_LAST) < COMMS_RATE)) return;
  if(compressData() < 0) logAlert("unable to compress Data", true);
  if(!sendSATCOMS()) logAlert("unable to communicate over RB", true);
  data.COMMS_LAST = millis();
}

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  uint64_t loopTime = millis() - data.LOOP_START;
  if (loopTime < LOOP_RATE) {
    gpsModule.smartDelay(LOOP_RATE - loopTime);
  }
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
 * Function: readHistory
 * -------------------
 * This function updates the data frame with values from EEPROM
 * if avionics is restarted mid flight.
 */
bool Avionics::readHistory() {
  if(!EEPROM.read(EEPROM_ROCKBLOCK)) data.RB_SHOULD_USE = false;
  if(!EEPROM.read(EEPROM_GPS)) data.GPS_SHOULD_USE = false;
  if(!EEPROM.read(EEPROM_HEATER)) data.HEATER_SHOULD_USE = false;
  double valveAltLast = PCB.readFromEEPROMAndClear(EEPROM_VALVE_START, EEPROM_VALVE_END);
  if (valveAltLast != 0) data.VALVE_ALT_LAST = valveAltLast;
  double ballastAltLast = PCB.readFromEEPROMAndClear(EEPROM_BALLAST_START, EEPROM_BALLAST_END);
  if (ballastAltLast != 0) data.BALLAST_ALT_LAST = ballastAltLast;
  return true;
}

/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
bool Avionics::readData() {
  data.TIME            = sensors.getTime();
  data.LOOP_RATE       = millis() - data.LOOP_START;
  data.LOOP_START      = millis();
  data.MINUTES         += (double)(((double)data.LOOP_RATE) / 1000.0 / 60.0);
  data.ALTITUDE_LAST   = data.ALTITUDE_BMP;
  data.VOLTAGE         = sensors.getVoltage();
  data.CURRENT         = sensors.getCurrent();
  data.CURRENT_GPS     = sensors.getCurrentGPS();
  data.CURRENT_RB      = sensors.getCurrentRB();
  data.CURRENT_MOTORS  = sensors.getCurrentMotors();
  data.CURRENT_PAYLOAD = sensors.getCurrentPayload();
  data.TEMP_NECK       = sensors.getNeckTemp();
  data.RAW_TEMP_1      = sensors.getRawTemp(1);
  data.RAW_TEMP_2      = sensors.getRawTemp(2);
  data.RAW_TEMP_3      = sensors.getRawTemp(3);
  data.RAW_TEMP_4      = sensors.getRawTemp(4);
  data.RAW_PRESSURE_1  = sensors.getRawPressure(1);
  data.RAW_PRESSURE_2  = sensors.getRawPressure(2);
  data.RAW_PRESSURE_3  = sensors.getRawPressure(3);
  data.RAW_PRESSURE_4  = sensors.getRawPressure(4);
  if ((millis() - data.GPS_LAST) >= GPS_RATE) {
    gpsModule.smartDelay(GPS_LOCK_TIME);
    data.LAT_GPS         = gpsModule.getLatitude();
    data.LONG_GPS        = gpsModule.getLongitude();
    data.ALTITUDE_GPS    = gpsModule.getAltitude();
    data.HEADING_GPS     = gpsModule.getCourse();
    data.SPEED_GPS       = gpsModule.getSpeed();
    data.NUM_SATS_GPS    = gpsModule.getSats();
    data.GPS_LAST = millis();
  }
  data.LOOP_GOOD_STATE = !data.LOOP_GOOD_STATE;
  return true;
}

/*
 * Function: processData
 * -------------------
 * This function updates the current data frame with derived values.
 */
bool Avionics::processData() {
  filter.enableSensors(data.BMP_1_ENABLE, data.BMP_2_ENABLE, data.BMP_3_ENABLE, data.BMP_4_ENABLE);
  data.RAW_ALTITUDE_1  = filter.getCalculatedAltitude(data.RAW_PRESSURE_1, data.PRESS_BASELINE);
  data.RAW_ALTITUDE_2  = filter.getCalculatedAltitude(data.RAW_PRESSURE_2, data.PRESS_BASELINE);
  data.RAW_ALTITUDE_3  = filter.getCalculatedAltitude(data.RAW_PRESSURE_3, data.PRESS_BASELINE);
  data.RAW_ALTITUDE_4  = filter.getCalculatedAltitude(data.RAW_PRESSURE_4, data.PRESS_BASELINE);
  data.TEMP            = filter.getTemp(data.RAW_TEMP_1, data.RAW_TEMP_2, data.RAW_TEMP_3, data.RAW_TEMP_4);
  data.PRESS_BMP       = filter.getPressure(data.RAW_PRESSURE_1, data.RAW_PRESSURE_2, data.RAW_PRESSURE_3, data.RAW_PRESSURE_4);
  data.ALTITUDE_BMP    = filter.getAltitude(data.RAW_ALTITUDE_1, data.RAW_ALTITUDE_2, data.RAW_ALTITUDE_3, data.RAW_ALTITUDE_4);
  data.ASCENT_RATE     = filter.getAscentRate();
  return true;
}

/*
 * Function: runHeaters
 * -------------------
 * This function thermally regulates the avionics. Disables heaters
 * if either the ballast or valve is running.
 */
bool Avionics::runHeaters() {
  if (!data.HEATER_SHOULD_USE || PCB.isValveRunning() || PCB.isBallastRunning()) {
    PCB.turnOffHeaters();
  } else {
    PCB.heater(data.TEMP_SETPOINT, data.TEMP);
  }
  return true;
}

/*
 * Function: runValve
 * -------------------
 * This function actuates the valve based on the calculated incentive.
 */
bool Avionics::runValve() {
  if(data.FORCE_VALVE || (data.VALVE_INCENTIVE >= 1)) {
    data.FORCE_VALVE = false;
    PCB.queueValve(VALVE_DURATION);
    data.VALVE_ALT_LAST = data.ALTITUDE_BMP;
    PCB.writeToEEPROM(EEPROM_VALVE_START, EEPROM_VALVE_END, data.ALTITUDE_BMP);
  }
  data.VALVE_STATE = PCB.checkValve();
  return true;
}

/*
 * Function: runBallast
 * -------------------
 * This function actuates the valve based on the calculated incentive.
 */
bool Avionics::runBallast() {
  if(data.FORCE_BALLAST || (data.BALLAST_INCENTIVE >= 1)) {
    data.FORCE_BALLAST = false;
    PCB.queueBallast(BALLAST_DURATION);
    data.BALLAST_ALT_LAST = data.ALTITUDE_BMP;
    PCB.writeToEEPROM(EEPROM_BALLAST_START, EEPROM_BALLAST_END, data.ALTITUDE_BMP);
  }
  data.BALLAST_STATE = PCB.checkBallast();
  return true;
}

/*
 * Function: runCutdown
 * -------------------
 * This function cuts down the payload if necessary.
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
  // split incoming string into up to eight commands
  const char* commandStrFormat = "%d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s";
  uint8_t commandIndexes[8] = {0};
  char commandStrings[8][100] = {{0},{0},{0},{0},{0},{0},{0},{0}};

  uint8_t numScanned = sscanf(COMMS_BUFFER, commandStrFormat,
                              commandIndexes[0], commandStrings[0],
                              commandIndexes[1], commandStrings[1],
                              commandIndexes[2], commandStrings[2],
                              commandIndexes[2], commandStrings[2],
                              commandIndexes[3], commandStrings[3],
                              commandIndexes[4], commandStrings[4],
                              commandIndexes[5], commandStrings[5],
                              commandIndexes[6], commandStrings[6],
                              commandIndexes[7], commandStrings[7]);

  // we should always gets pairs of indexes and command strings
  if (numScanned % 2 != 0) return;

  // parse each command
  for (uint8_t i = 0; i < numScanned / 2; i++) {
    uint8_t index = commandIndexes[i];

    // if index and command string match cutdown command, tell hardware to cut down
    if (index == CUTDOWN_INDEX && strncmp(commandStrings[i], CUTDOWN_COMMAND, strlen(commandStrings[i])) == 0) {
      data.SHOULD_CUTDOWN = true;
    }

    // if index is out of bounds, stop parsing
    if (index < 0 || index > 80) return;

    // otherwise, convert the command into a numerical value
    char* charAfterNumbers;
    float commandValue = (float) strtod(commandStrings[i], &charAfterNumbers);
    if (*charAfterNumbers) return; // if there are non-numeric characters after our number, then invalid command value; stop parsing
    updateConstant(index, commandValue);
  }
}

void Avionics::updateConstant(uint8_t index, float value) {
  if (index == 0) data.VALVE_ALT_LAST = value;
  else if (index == 1) data.BALLAST_ALT_LAST = value;

  else if (index == 2) data.VALVE_SETPOINT = value;
  else if (index == 3) data.BALLAST_SETPOINT = value;
  else if (index == 4) data.TEMP_SETPOINT = value;
  // else if (index == 5) ALTITUDE_CHANGE_BALLAST = value; // TODO: ???

  else if (index == 6) data.VALVE_VELOCITY_CONSTANT = value;
  else if (index == 7) data.VALVE_ALTITUDE_DIFF_CONSTANT = 1.0 / value;
  else if (index == 8) data.VALVE_LAST_ACTION_CONSTANT = 1.0 / value;
  else if (index == 9) data.BALLAST_VELOCITY_CONSTANT = value;
  else if (index == 10) data.BALLAST_ALTITUDE_DIFF_CONSTANT = 1.0 / value;
  else if (index == 11) data.BALLAST_LAST_ACTION_CONSTANT = 1.0 / value;

  // TODO: not sure if we have equivalents for these four values here
  // else if (index == 12) COMM_BEACON_INTERVAL = valuee;
  // else if (index == 13) GPS_BEACON_INTERVAL = valuee;
  // else if (index == 14) IridiumVentTime = valuee;
  // else if (index == 15) IridiumBallastTime = valuee;
  // else if (index == 16) activeSensors = (int)valuee;

  else if (index == 17) parseAvionicsModeCommand((int) value);
  else if (index == 30) data.FORCE_VALVE = true;
  else if (index == 31) data.FORCE_BALLAST = true;
  // else if (index == 33) LEDon = (bool) valuee; // TODO: we don't use this, right?
  else if (index == 34) parseRockBlockCommand((bool) value);
  else if (index == 35) parseGPSCommand((int) value);
  else if (index == 36) parseHeaterCommand((bool) value);

  else if (index == 37) data.PRESS_BASELINE = value;
  // else if (index == 38) DO_NOTHING_TIMER = valuee; // TODO: we don't use this, right?
  else if (index == 42) VALVE_MOTOR_SPEED = value;
  // else if (index == 43) VALVE_OPEN_BACKUP_TIMER = valuee; // TODO: we don't use this, right?
  else if (index == 44) parseHeaterModeCommand((int) value);
  else if (index == 45) data.INCENTIVE_THRESHOLD = value;
}

// TODO: I don't understand what the 0, 2, and else mode settings mean
void Avionics::parseAvionicsModeCommand(int command) {
  if (command == 0) {          // bad things?
    data.REPORT_MODE = false;
    data.CONTROL_MODE = false;
  } else if (command == 1) {   // normal flight mode
    data.REPORT_MODE = false;
    data.CONTROL_MODE = true;
  } else if (command == 2) {   // more bad things?
    data.REPORT_MODE = true;
    data.CONTROL_MODE = false;
  } else if (command == 3) {   // launch / data gathering mode
    data.REPORT_MODE = true;
    data.CONTROL_MODE = true;
  } else {                     // more bad things?
    data.REPORT_MODE = true;
    data.CONTROL_MODE = false;
  }
}

void Avionics::parseRockBlockCommand(bool command) {

}
void Avionics::parseGPSCommand(int command) {

}
void Avionics::parseHeaterCommand(bool command) {

}
void Avionics::parseHeaterModeCommand(int command) {

}

  /* TODO***********************************************************************
    data.VALVE_INCENTIVE
    data.BALLAST_INCENTIVE
    data.CONTROL_MODE
    data.REPORT_MODE
    data.SHOULD_CUTDOWN
    data.PRESS_BASELINE
    data.TEMP_SETPOINT
    data.INCENTIVE_THRESHOLD
    data.RE_ARM_CONSTANT
    data.VALVE_SETPOINT
    data.VALVE_TIME
    data.VALVE_ALT_LAST
    data.VALVE_VELOCITY_CONSTANT
    data.VALVE_ALTITUDE_DIFF_CONSTANT
    data.VALVE_LAST_ACTION_CONSTANT
    data.BALLAST_SETPOINT
    data.BALLAST_TIME
    data.BALLAST_ALT_LAST
    data.BALLAST_VELOCITY_CONSTANT
    data.BALLAST_ALTITUDE_DIFF_CONSTANT
    data.BALLAST_LAST_ACTION_CONSTANT
    data.RB_SHOULD_USE
    data.GPS_SHOULD_USE
    data.HEATER_SHOULD_USE
    data.BMP_1_ENABLE
    data.BMP_2_ENABLE
    data.BMP_3_ENABLE
    data.BMP_4_ENABLE
    data.SETUP_STATE
    data.FORCE_VALVE
    data.FORCE_BALLAST
  */

/*
 * Function: calcVitals
 * -------------------
 * This function calculates if the current state is within bounds.
 */
bool Avionics::calcVitals() {
  data.BAT_GOOD_STATE  = (data.VOLTAGE >= 3.63);
  data.CURR_GOOD_STATE = (data.CURRENT > -5.0 && data.CURRENT <= 500.0);
  data.PRES_GOOD_STATE = (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200);
  data.TEMP_GOOD_STATE = (data.TEMP > 15 && data.TEMP < 50);
  data.GPS_GOOD_STATE  = (data.LAT_GPS != 1000.0 && data.LAT_GPS != 0.0 && data.LONG_GPS != 1000.0 && data.LONG_GPS != 0.0);
  return true;
}

/*
 * Function: calcDebug
 * -------------------
 * This function calculates if the avionics is in debug mode.
 */
bool Avionics::calcDebug() {
  if(data.DEBUG_STATE   && (data.ALTITUDE_LAST >= DEBUG_ALT) && (data.ALTITUDE_BMP >= DEBUG_ALT)) {
    data.DEBUG_STATE = false;
  }
  return true;
}

/*
 * Function: calcIncentives
 * -------------------
 * This function gets the updated incentives from the flight computer.
 */
bool Avionics::calcIncentives() {
  computer.updateControllerConstants(data.INCENTIVE_THRESHOLD, data.RE_ARM_CONSTANT);
  computer.updateValveConstants(data.VALVE_SETPOINT, data.VALVE_VELOCITY_CONSTANT, data.VALVE_ALTITUDE_DIFF_CONSTANT, data.VALVE_LAST_ACTION_CONSTANT);
  computer.updateBallastConstants(data.BALLAST_SETPOINT, data.BALLAST_VELOCITY_CONSTANT, data.BALLAST_ALTITUDE_DIFF_CONSTANT, data.BALLAST_LAST_ACTION_CONSTANT);
  data.VALVE_INCENTIVE   = computer.getValveIncentive(data.ASCENT_RATE, data.ALTITUDE_BMP, data.VALVE_ALT_LAST);
  data.BALLAST_INCENTIVE = computer.getBallastIncentive(data.ASCENT_RATE, data.ALTITUDE_BMP, data.BALLAST_ALT_LAST);
  if (data.VALVE_INCENTIVE >= 1 && data.BALLAST_INCENTIVE >= 1) {
    data.VALVE_INCENTIVE = 0;
    data.BALLAST_INCENTIVE = 0;
    return -1;
  }
  return true;
}

/*
 * Function: calcCutdown
 * -------------------
 * This function calculates if the avionics should cutdown.
 */
bool Avionics::calcCutdown() {
  if(CUTDOWN_GPS_ENABLE && data.GPS_GOOD_STATE &&
    (((data.LAT_GPS < GPS_FENCE_LAT_MIN) || (data.LAT_GPS > GPS_FENCE_LAT_MAX)) ||
    ((data.LONG_GPS < GPS_FENCE_LON_MIN) || (data.LONG_GPS > GPS_FENCE_LON_MAX)))
  ) data.SHOULD_CUTDOWN  = true;

  if(CUTDOWN_ALT_ENABLE && !data.CUTDOWN_STATE &&
    (data.ALTITUDE_LAST >= CUTDOWN_ALT) &&
    (data.ALTITUDE_BMP  >= CUTDOWN_ALT)
  ) data.SHOULD_CUTDOWN  = true;
  return true;
}

/*
 * Function: debugState
 * -------------------
 * This function provides debuging information.
 */
bool Avionics::debugState() {
  if(data.DEBUG_STATE) printState();
  return true;
}

/*
 * Function: setupLog
 * -------------------
 * This function initializes the SD card file.
 */
void Avionics::setupLog() {
  Serial.println("Card Initialitzed");
  char filename[] = "LOGGER00.txt";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      break;
    }
  }
  logFile = SD.open("EVENTS.txt", FILE_WRITE);
  if (!dataFile || !logFile) {
    PCB.faultLED();
    Serial.println ("ERROR: COULD NOT CREATE FILE");
  }
  else {
    Serial.print("Logging to: ");
    Serial.println(filename);
  }
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
  dataFile.print("Stanford Student Space Initiative Balloons Launch ");
  dataFile.print(MISSION_NUMBER);
  dataFile.print('\n');
  dataFile.print(CSV_DATA_HEADER);
  dataFile.print('\n');
  dataFile.flush();
}

/*
 * Function: logAlert
 * -------------------
 * This function logs important information whenever a specific event occurs.
 */
void Avionics::logAlert(const char* debug, bool fatal) {
  if(fatal) PCB.faultLED();
  if(logFile) {
    logFile.print(data.TIME);
    logFile.print(',');
    if(fatal) logFile.print("FATAL ERROR!!!!!!!!!!: ");
    else logFile.print("Alert: ");
    logFile.print(debug);
    logFile.print("...\n");
    logFile.flush();
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
 * Function: compressVariable
 * -------------------
 * This function compresses a single variable into a scaled digital bitmask.
 */
int16_t Avionics::compressVariable(float var, float minimum, float maximum, int16_t resolution, int16_t length) {
  if (resolution <= 0) return -1;
  if (var < minimum) var = minimum;
  if (var > maximum) var = maximum;
  int32_t adc = round( (pow(2, resolution) - 1) * (var - minimum) / (maximum - minimum));
  int16_t byteIndex = length / 8;
  int16_t bitIndex = 7 - (length % 8);
  for (int16_t i = resolution - 1; i >= 0; i--) {
    bool bit = adc & (1 << i);
    if (bit) COMMS_BUFFER[byteIndex] |= (1 << bitIndex);
    bitIndex -= 1;
    if (bitIndex == 0) {
      bitIndex = 7;
      byteIndex++;
    }
  }
  return resolution;
}

/*
 * Function: printState
 * -------------------
 * This function prints the current avionics state.
 */
void Avionics::printState() {
  Serial.print(data.TIME);
  Serial.print(',');
  Serial.print(data.MINUTES, 6);
  Serial.print(',');
  Serial.print(data.ALTITUDE_BMP);
  Serial.print(',');
  Serial.print(data.ASCENT_RATE);
  Serial.print(',');
  Serial.print(data.VALVE_INCENTIVE);
  Serial.print(',');
  Serial.print(data.BALLAST_INCENTIVE);
  Serial.print(',');
  Serial.print(data.TEMP);
  Serial.print(',');
  Serial.print(data.TEMP_NECK);
  Serial.print(',');
  Serial.print(data.VOLTAGE);
  Serial.print(',');
  Serial.print(data.CURRENT * 1000);
  Serial.print(',');
  Serial.print(data.JOULES);
  Serial.print(',');
  Serial.print(data.HEATER_PWM);
  Serial.print(',');
  Serial.print(data.LAT_GPS, 4);
  Serial.print(',');
  Serial.print(data.LONG_GPS, 4);
  Serial.print(',');
  Serial.print(data.LOOP_RATE);
  Serial.print(',');
  Serial.print(data.RB_SENT_COMMS);
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
  Serial.print(data.CURRENT_GPS);
  Serial.print(',');
  Serial.print(data.CURRENT_RB);
  Serial.print(',');
  Serial.print(data.CURRENT_MOTORS);
  Serial.print(',');
  Serial.print(data.CURRENT_PAYLOAD);
  Serial.print(',');
  Serial.print(data.INCENTIVE_THRESHOLD);
  Serial.print(',');
  Serial.print(data.RE_ARM_CONSTANT);
  Serial.print(',');
  Serial.print(data.VALVE_SETPOINT);
  Serial.print(',');
  Serial.print(data.VALVE_TIME);
  Serial.print(',');
  Serial.print(data.VALVE_ALT_LAST);
  Serial.print(',');
  Serial.print(data.VALVE_VELOCITY_CONSTANT);
  Serial.print(',');
  Serial.print(data.VALVE_ALTITUDE_DIFF_CONSTANT);
  Serial.print(',');
  Serial.print(data.VALVE_LAST_ACTION_CONSTANT);
  Serial.print(',');
  Serial.print(data.BALLAST_SETPOINT);
  Serial.print(',');
  Serial.print(data.BALLAST_TIME);
  Serial.print(',');
  Serial.print(data.BALLAST_ALT_LAST);
  Serial.print(',');
  Serial.print(data.BALLAST_VELOCITY_CONSTANT);
  Serial.print(',');
  Serial.print(data.BALLAST_ALTITUDE_DIFF_CONSTANT);
  Serial.print(',');
  Serial.print(data.BALLAST_LAST_ACTION_CONSTANT);
  Serial.print(',');
  Serial.print(data.RB_SHOULD_USE);
  Serial.print(',');
  Serial.print(data.GPS_SHOULD_USE);
  Serial.print(',');
  Serial.print(data.HEATER_SHOULD_USE);
  Serial.print(',');
  Serial.print(data.CUTDOWN_STATE);
  Serial.print(',');
  Serial.print(data.ALTITUDE_LAST);
  Serial.print(',');
  Serial.print(double(data.GPS_LAST));
  Serial.print(',');
  Serial.print(double(data.COMMS_LAST));
  Serial.print(',');
  Serial.print(double(data.LOOP_START));
  Serial.print(',');
  Serial.print(data.CONTROL_MODE);
  Serial.print(',');
  Serial.print(data.REPORT_MODE);
  Serial.print(',');
  Serial.print(data.COMMS_LENGTH);
  Serial.print(',');
  Serial.print(data.SHOULD_CUTDOWN);
  Serial.print(',');
  Serial.print(data.PRESS_BASELINE);
  Serial.print(',');
  Serial.print(data.TEMP_SETPOINT);
  Serial.print(',');
  Serial.print(data.BMP_1_ENABLE);
  Serial.print(',');
  Serial.print(data.BMP_2_ENABLE);
  Serial.print(',');
  Serial.print(data.BMP_3_ENABLE);
  Serial.print(',');
  Serial.print(data.BMP_4_ENABLE);
  Serial.print(',');
  Serial.print(data.SETUP_STATE);
  Serial.print(',');
  Serial.print(data.VALVE_STATE);
  Serial.print(',');
  Serial.print(data.BALLAST_STATE);
  Serial.print(',');
  Serial.print(data.FORCE_VALVE);
  Serial.print(',');
  Serial.print(data.FORCE_BALLAST);
  Serial.print(',');
  Serial.print(data.BAT_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.CURR_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.PRES_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.TEMP_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.CAN_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.RB_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.GPS_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.LOOP_GOOD_STATE);
  Serial.print(',');
  Serial.print(data.RAW_TEMP_1);
  Serial.print(',');
  Serial.print(data.RAW_TEMP_2);
  Serial.print(',');
  Serial.print(data.RAW_TEMP_3);
  Serial.print(',');
  Serial.print(data.RAW_TEMP_4);
  Serial.print(',');
  Serial.print(data.RAW_PRESSURE_1);
  Serial.print(',');
  Serial.print(data.RAW_PRESSURE_2);
  Serial.print(',');
  Serial.print(data.RAW_PRESSURE_3);
  Serial.print(',');
  Serial.print(data.RAW_PRESSURE_4);
  Serial.print(',');
  Serial.print(data.RAW_ALTITUDE_1);
  Serial.print(',');
  Serial.print(data.RAW_ALTITUDE_2);
  Serial.print(',');
  Serial.print(data.RAW_ALTITUDE_3);
  Serial.print(',');
  Serial.print(data.RAW_ALTITUDE_4);
  Serial.print('\n');
}

/*
 * Function: logData
 * -------------------
 * This function logs the current data frame.
 */
bool Avionics::logData() {
  bool sucess = true;
  dataFile.print(data.TIME);
  dataFile.print(',');
  dataFile.print(data.MINUTES, 6);
  dataFile.print(',');
  dataFile.print(data.ALTITUDE_BMP);
  dataFile.print(',');
  dataFile.print(data.ASCENT_RATE);
  dataFile.print(',');
  dataFile.print(data.VALVE_INCENTIVE);
  dataFile.print(',');
  dataFile.print(data.BALLAST_INCENTIVE);
  dataFile.print(',');
  dataFile.print(data.TEMP);
  dataFile.print(',');
  dataFile.print(data.TEMP_NECK);
  dataFile.print(',');
  dataFile.print(data.VOLTAGE);
  dataFile.print(',');
  dataFile.print(data.CURRENT * 1000);
  dataFile.print(',');
  dataFile.print(data.JOULES);
  dataFile.print(',');
  dataFile.print(data.HEATER_PWM);
  dataFile.print(',');
  dataFile.print(data.LAT_GPS, 4);
  dataFile.print(',');
  dataFile.print(data.LONG_GPS, 4);
  dataFile.print(',');
  dataFile.print(data.LOOP_RATE);
  dataFile.print(',');
  dataFile.print(data.RB_SENT_COMMS);
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
  dataFile.print(data.CURRENT_GPS);
  dataFile.print(',');
  dataFile.print(data.CURRENT_RB);
  dataFile.print(',');
  dataFile.print(data.CURRENT_MOTORS);
  dataFile.print(',');
  dataFile.print(data.CURRENT_PAYLOAD);
  dataFile.print(',');
  dataFile.print(data.INCENTIVE_THRESHOLD);
  dataFile.print(',');
  dataFile.print(data.RE_ARM_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.VALVE_SETPOINT);
  dataFile.print(',');
  dataFile.print(data.VALVE_TIME);
  dataFile.print(',');
  dataFile.print(data.VALVE_ALT_LAST);
  dataFile.print(',');
  dataFile.print(data.VALVE_VELOCITY_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.VALVE_ALTITUDE_DIFF_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.VALVE_LAST_ACTION_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.BALLAST_SETPOINT);
  dataFile.print(',');
  dataFile.print(data.BALLAST_TIME);
  dataFile.print(',');
  dataFile.print(data.BALLAST_ALT_LAST);
  dataFile.print(',');
  dataFile.print(data.BALLAST_VELOCITY_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.BALLAST_ALTITUDE_DIFF_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.BALLAST_LAST_ACTION_CONSTANT);
  dataFile.print(',');
  dataFile.print(data.RB_SHOULD_USE);
  dataFile.print(',');
  dataFile.print(data.GPS_SHOULD_USE);
  dataFile.print(',');
  dataFile.print(data.HEATER_SHOULD_USE);
  dataFile.print(',');
  dataFile.print(data.CUTDOWN_STATE);
  dataFile.print(',');
  dataFile.print(data.ALTITUDE_LAST);
  dataFile.print(',');
  dataFile.print(double(data.GPS_LAST));
  dataFile.print(',');
  dataFile.print(double(data.COMMS_LAST));
  dataFile.print(',');
  dataFile.print(double(data.LOOP_START));
  dataFile.print(',');
  dataFile.print(data.CONTROL_MODE);
  dataFile.print(',');
  dataFile.print(data.REPORT_MODE);
  dataFile.print(',');
  dataFile.print(data.COMMS_LENGTH);
  dataFile.print(',');
  dataFile.print(data.SHOULD_CUTDOWN);
  dataFile.print(',');
  dataFile.print(data.PRESS_BASELINE);
  dataFile.print(',');
  dataFile.print(data.TEMP_SETPOINT);
  dataFile.print(',');
  dataFile.print(data.BMP_1_ENABLE);
  dataFile.print(',');
  dataFile.print(data.BMP_2_ENABLE);
  dataFile.print(',');
  dataFile.print(data.BMP_3_ENABLE);
  dataFile.print(',');
  dataFile.print(data.BMP_4_ENABLE);
  dataFile.print(',');
  dataFile.print(data.SETUP_STATE);
  dataFile.print(',');
  dataFile.print(data.VALVE_STATE);
  dataFile.print(',');
  dataFile.print(data.BALLAST_STATE);
  dataFile.print(',');
  dataFile.print(data.FORCE_VALVE);
  dataFile.print(',');
  dataFile.print(data.FORCE_BALLAST);
  dataFile.print(',');
  dataFile.print(data.BAT_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.CURR_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.PRES_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.TEMP_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.CAN_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.RB_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.GPS_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.LOOP_GOOD_STATE);
  dataFile.print(',');
  dataFile.print(data.RAW_TEMP_1);
  dataFile.print(',');
  dataFile.print(data.RAW_TEMP_2);
  dataFile.print(',');
  dataFile.print(data.RAW_TEMP_3);
  dataFile.print(',');
  dataFile.print(data.RAW_TEMP_4);
  dataFile.print(',');
  dataFile.print(data.RAW_PRESSURE_1);
  dataFile.print(',');
  dataFile.print(data.RAW_PRESSURE_2);
  dataFile.print(',');
  dataFile.print(data.RAW_PRESSURE_3);
  dataFile.print(',');
  dataFile.print(data.RAW_PRESSURE_4);
  dataFile.print(',');
  dataFile.print(data.RAW_ALTITUDE_1);
  dataFile.print(',');
  dataFile.print(data.RAW_ALTITUDE_2);
  dataFile.print(',');
  dataFile.print(data.RAW_ALTITUDE_3);
  if(dataFile.print(',') != 1) sucess = false;
  dataFile.print(data.RAW_ALTITUDE_4);
  dataFile.print('\n');
  dataFile.flush();
  return sucess;
}

/*
 * Function: compressData
 * -------------------
 * This function compresses the data frame into a bit stream.
 */
int16_t Avionics::compressData() {
  int16_t lengthBits  = 0;
  int16_t lengthBytes = 0;
  for(uint16_t i = 0; i < BUFFER_SIZE; i++) COMMS_BUFFER[i] = 0;
  lengthBits += compressVariable(data.TIME,                           0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.MINUTES,                        0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.ALTITUDE_BMP,                  -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.ASCENT_RATE,                   -10,   10,      11, lengthBits);
  lengthBits += compressVariable(data.VALVE_INCENTIVE,               -50,   10,      12, lengthBits);
  lengthBits += compressVariable(data.BALLAST_INCENTIVE,             -50,   10,      12, lengthBits);
  lengthBits += compressVariable(data.TEMP,                          -50,   100,     9,  lengthBits);
  lengthBits += compressVariable(data.TEMP_NECK,                     -50,   100,     9,  lengthBits);
  lengthBits += compressVariable(data.VOLTAGE,                        0,    5,       9,  lengthBits);
  lengthBits += compressVariable(data.CURRENT,                        0,    5000,    8,  lengthBits);
  lengthBits += compressVariable(data.JOULES,                         0,    1500000, 18, lengthBits);
  lengthBits += compressVariable(data.HEATER_PWM,                     0,    255,     8,  lengthBits);
  lengthBits += compressVariable(data.LAT_GPS,                       -90,   90,      21, lengthBits);
  lengthBits += compressVariable(data.LONG_GPS,                      -180,  180,     22, lengthBits);
  lengthBits += compressVariable(data.LOOP_RATE,                      0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.RB_SENT_COMMS,                  0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.SPEED_GPS,                     -100,  100,     9,  lengthBits);
  lengthBits += compressVariable(data.HEADING_GPS,                   -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.ALTITUDE_GPS,                  -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.PRESS_BMP,                      0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.NUM_SATS_GPS,                   0,    10,      11, lengthBits);
  lengthBits += compressVariable(data.CURRENT_GPS,                    0,    1000,    6,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_RB,                     0,    1000,    6,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTORS,                 0,    1000,    6,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_PAYLOAD,                0,    1000,    6,  lengthBits);
  lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,            0,    4,       8,  lengthBits);
  lengthBits += compressVariable(data.RE_ARM_CONSTANT,                0,    4,       8,  lengthBits);
  lengthBits += compressVariable(data.VALVE_SETPOINT,                -2000, 50000,   11, lengthBits);
  lengthBits += compressVariable(data.VALVE_TIME,                     0,    50,      6,  lengthBits);
  lengthBits += compressVariable(data.VALVE_ALT_LAST,                -2000, 50000,   11, lengthBits);
  lengthBits += compressVariable(data.VALVE_VELOCITY_CONSTANT,        0,    1000,    8,  lengthBits);
  lengthBits += compressVariable(data.VALVE_ALTITUDE_DIFF_CONSTANT,   0,    4000,    8,  lengthBits);
  lengthBits += compressVariable(data.VALVE_LAST_ACTION_CONSTANT,     0,    4000,    8,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_SETPOINT,              -2000, 50000,   11, lengthBits);
  lengthBits += compressVariable(data.BALLAST_TIME,                   0,    50,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_ALT_LAST,              -2000, 50000,   11, lengthBits);
  lengthBits += compressVariable(data.BALLAST_VELOCITY_CONSTANT,      0,    1000,    8,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_ALTITUDE_DIFF_CONSTANT, 0,    4000,    8,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_LAST_ACTION_CONSTANT,   0,    4000,    8,  lengthBits);
  lengthBits += compressVariable(data.RB_SHOULD_USE,                  0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.GPS_SHOULD_USE,                 0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.HEATER_SHOULD_USE,              0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.CUTDOWN_STATE,                  0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.ALTITUDE_LAST,                 -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.GPS_LAST,                       0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.COMMS_LAST,                     0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.LOOP_START,                     0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.CONTROL_MODE,                   0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.REPORT_MODE,                    0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.COMMS_LENGTH,                   0,    200,     8,  lengthBits);
  lengthBits += compressVariable(data.SHOULD_CUTDOWN,                 0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.PRESS_BASELINE,                 0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.TEMP_SETPOINT,                 -20,   40,      6,  lengthBits);
  lengthBits += compressVariable(data.BMP_1_ENABLE,                   0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BMP_2_ENABLE,                   0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BMP_3_ENABLE,                   0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BMP_4_ENABLE,                   0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.SETUP_STATE,                    0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.DEBUG_STATE,                    0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.VALVE_STATE,                    0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_STATE,                  0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.FORCE_VALVE,                    0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.FORCE_BALLAST,                  0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BAT_GOOD_STATE,                 0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.CURR_GOOD_STATE,                0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.PRES_GOOD_STATE,                0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.TEMP_GOOD_STATE,                0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.CAN_GOOD_STATE,                 0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.RB_GOOD_STATE,                  0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.GPS_GOOD_STATE,                 0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.LOOP_GOOD_STATE,                0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.RAW_TEMP_1,                    -50,   100,     9,  lengthBits);
  lengthBits += compressVariable(data.RAW_TEMP_2,                    -50,   100,     9,  lengthBits);
  lengthBits += compressVariable(data.RAW_TEMP_3,                    -50,   100,     9,  lengthBits);
  lengthBits += compressVariable(data.RAW_TEMP_4,                    -50,   100,     9,  lengthBits);
  lengthBits += compressVariable(data.RAW_PRESSURE_1,                 0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.RAW_PRESSURE_2,                 0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.RAW_PRESSURE_3,                 0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.RAW_PRESSURE_4,                 0,    1000000, 19, lengthBits);
  lengthBits += compressVariable(data.RAW_ALTITUDE_1,                -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.RAW_ALTITUDE_2,                -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.RAW_ALTITUDE_3,                -2000, 40000,   16, lengthBits);
  lengthBits += compressVariable(data.RAW_ALTITUDE_4,                -2000, 40000,   16, lengthBits);
  lengthBits += 8 - (lengthBits % 8);
  lengthBytes = lengthBits / 8;
  data.COMMS_LENGTH = lengthBytes;
  return lengthBytes;
}
