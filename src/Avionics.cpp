/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Keegan Mehall | kmehall@stanford.edu

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
  if(!setupSDCard())                                            alert("unable to initialize SD Card", true);
  if(!readHistory())                                            alert("unable to initialize EEPROM", true);
  if(!sensors.init())                                           alert("unable to initialize Sensors", true);
#ifdef HITL_ENABLED_FLAG
  if(!HITL.init())                                              alert("unable to initialize Simulations", true);
#endif
  if(!filter.init())                                            alert("unable to initialize Filters", true);
  if(!computer.init())                                          alert("unable to initialize Flight Controller", true);
  if(!gpsModule.init(data.POWER_STATE_GPS))                     alert("unable to initialize GPS", true);
#ifndef RB_DISABLED_FLAG
  if(!RBModule.init(data.POWER_STATE_RB, data.RB_SHOULD_SLEEP)) alert("unable to initialize RockBlock", true);
#endif
  if(!PCB.startUpHeaters(data.POWER_STATE_HEATER))              alert("unable to initialize Heaters", true);
  if(!ValMU.init(data.POWER_STATE_PAYLOAD))                     alert("unable to initialize Payload", true);
  PCB.initResolutions();
  data.TIME = millis();
  data.SETUP_STATE = false;
}

/*
 * Function: test
 * -------------------
 * This function tests the hardware.
 */
void Avionics::test() {
  Serial.println("test starting");
  data.MANUAL_MODE = false;
  // uint32_t startt = millis();
  // data.VALVE_LEAK_INTERVAL = 10000;
  // while((millis() - startt) <= (1000 * 60 * 2)) {
  //   data.CURRENT_MOTOR_VALVE = (data.VALVE_STATE ? sensors.getCurrentSubsystem(MOTORS_CURRENT) : 0);
  //   data.VALVE_QUEUE = PCB.getValveQueue();
  //   data.VALVE_STATE = PCB.checkValve(data.CURRENT_MOTOR_VALVE, data.VALVE_LEAK_INTERVAL);
  // }
  data.SHOULD_CUTDOWN = true;
  //PCB.queueBallast(60000, true);
  //PCB.queueValve(5000, true);
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateState
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateState() {
#ifndef HITL_ENABLED_FLAG
  if(!readData())     alert("unable to read Data", true);
#endif
#ifdef HITL_ENABLED_FLAG
  if(!simulateData()) alert("unable to simulate Data", true);
#endif
  if(!processData())    alert("unable to process Data", true);
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently calculates the current state.
 */
void Avionics::evaluateState() {
  if(!calcVitals())     alert("unable to calculate vitals", true);
  if(!calcDebug())      alert("unable to calculate debug", true);
  if(!calcIncentives()) alert("unable to calculate incentives", true);
}

/*
 * Function: actuateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::actuateState() {
  if(!runHeaters()) alert("unable to run heaters", true);
  if(!runValve())   alert("unable to run valve", true);
  if(!runBallast()) alert("unable to run ballast", true);
  if(!runCutdown()) alert("unable to run cutdown", true);
  if(!runLED())     alert("unable to run LED", true);
}

/*
 * Function: logState
 * -------------------
 * This function logs the current data frame.
 */
void Avionics::logState() {
  uint32_t t0 = millis();
  if(!log.log(&data, PCB.valveState != PCB.OPENING)) alert("unable to log Data", true);
  data.LOG_TIME = millis() - t0;
  data.LOOP_NUMBER2++;
  if(!debugState())   alert("unable to debug state", true);
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
#ifndef RB_DISABLED_FLAG
  data.RB_WAKE_FAILS = RBModule.getNumWakeFailures();
  data.RB_SLEEP_FAILS = RBModule.getNumSleepFailures();
  if(!data.VALVE_STATE && !data.POWER_STATE_RB && ((millis() - data.RB_LAST) > RB_RESTART_INTERVAL)) {
    data.RB_SHOULD_SLEEP = false;
    RBModule.restart(data.RB_SHOULD_SLEEP);
  }
  if(data.DEBUG_STATE && ((millis() - data.RB_LAST) < RB_DEBUG_INTERVAL)) return;
  if(!data.DEBUG_STATE && ((millis() - data.RB_LAST) < data.RB_INTERVAL)) return;
  if(compressData() < 0) alert("unable to compress Data", true);
  if(!sendSATCOMS())     alert("unable to communicate over RB", true);
  else data.RB_LAST = millis();
#endif
}

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  uint32_t loopTime = millis() - data.TIME;
  if (loopTime < LOOP_INTERVAL) gpsModule.smartDelay(LOOP_INTERVAL - loopTime);
  data.LOOP_NUMBER++;
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
 * Function: setupSDCard
 * -------------------
 * This function sets up the SD card for logging.
 */
bool Avionics::setupSDCard() {
  log.initialize();

  #ifdef RESET_EEPROM_FLAG
    PCB.EEPROMWritelong(EEPROM_LOG_BLOCK_CUR, 0);
    PCB.EEPROMWritelong(EEPROM_LOG_FILE_NUM, 0);
  #endif

  return log.setupLogfile();
}

/*
 * Function: readHistory
 * -------------------
 * This function updates the data frame with values from EEPROM
 * if avionics is restarted mid flight.
 */
bool Avionics::readHistory() {
#ifdef RESET_EEPROM_FLAG
  for(size_t i = 0; i < 1023; i++)   EEPROM.write(i, 0x0);
    EEPROM.write(EEPROM_ROCKBLOCK, true);
    EEPROM.write(EEPROM_GPS, true);
    EEPROM.write(EEPROM_HEATER, true);
    EEPROM.write(EEPROM_PAYLOAD, true);
    PCB.EEPROMWritelong(EEPROM_VALVE_ALT_LAST, data.VALVE_ALT_LAST);
    PCB.EEPROMWritelong(EEPROM_BALLAST_ALT_LAST, data.BALLAST_ALT_LAST);
#endif
#ifndef RESET_EEPROM_FLAG
  if(!EEPROM.read(EEPROM_ROCKBLOCK)) data.POWER_STATE_RB = false;
  if(!EEPROM.read(EEPROM_GPS)) data.POWER_STATE_GPS = false;
  if(!EEPROM.read(EEPROM_HEATER)) data.POWER_STATE_HEATER = false;
  if(!EEPROM.read(EEPROM_PAYLOAD)) data.POWER_STATE_PAYLOAD = false;
  double valveAltLast = PCB.EEPROMReadlong(EEPROM_VALVE_ALT_LAST);
  if (valveAltLast != 0) data.VALVE_ALT_LAST = valveAltLast;
  double ballastAltLast = PCB.EEPROMReadlong(EEPROM_BALLAST_ALT_LAST);
  if (ballastAltLast != 0) data.BALLAST_ALT_LAST = ballastAltLast;
#endif
  return true;
}

/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
bool Avionics::readData() {
  data.LOOP_TIME             = millis() - data.TIME;
  data.TIME                  = millis();
  data.VOLTAGE_PRIMARY       = sensors.getVoltagePrimary();
  data.VOLTAGE_5V            = sensors.getVoltage5V();
  data.CURRENT_USB           = sensors.getCurrentUSB();
  data.CURRENT_TOTAL         = sensors.getCurrentTotal();
  data.JOULES_TOTAL          = sensors.getJoules();
  data.PID_OUTPUT            = PCB.getHeaterPID();
  data.JOULES_HEATER         = sensors.getJoulesHeater(data.PID_OUTPUT, data.HEATER_STRONG_ENABLE, data.HEATER_WEEK_ENABLE);
  data.CURRENT_RB            = sensors.getCurrentSubsystem(RB_CURRENT);
  data.CURRENT_MOTOR_VALVE   = (data.VALVE_STATE ? sensors.getCurrentSubsystem(MOTORS_CURRENT) : 0);
  data.CURRENT_MOTOR_BALLAST = (data.BALLAST_STATE ? sensors.getCurrentSubsystem(MOTORS_CURRENT) : 0);
  data.CURRENT_PAYLOAD       = sensors.getCurrentSubsystem(PAYLOAD_CURRENT);
  data.RAW_TEMP_1            = sensors.getRawTemp(1);
  data.RAW_TEMP_2            = sensors.getRawTemp(2);
  data.RAW_TEMP_3            = sensors.getRawTemp(3);
  data.RAW_TEMP_4            = sensors.getRawTemp(4);
  data.RAW_PRESSURE_1        = sensors.getRawPressure(1);
  data.RAW_PRESSURE_2        = sensors.getRawPressure(2);
  data.RAW_PRESSURE_3        = sensors.getRawPressure(3);
  data.RAW_PRESSURE_4        = sensors.getRawPressure(4);
  if (data.POWER_STATE_GPS && ((millis() - data.GPS_LAST) >= data.GPS_INTERVAL) && (!data.VALVE_STATE)) readGPS();
  if (data.POWER_STATE_PAYLOAD) readPayload();
  return true;
}

/*
 * Function: readGPS
 * -------------------
 * This function reads data from the GPS module.
 */
bool Avionics::readGPS() {
  gpsModule.smartDelay(GPS_LOCK_TIMEOUT);
  data.LAT_GPS          = gpsModule.getLatitude();
  data.LONG_GPS         = gpsModule.getLongitude();
  data.ALTITUDE_GPS     = gpsModule.getAltitude();
  data.HEADING_GPS      = gpsModule.getCourse();
  data.SPEED_GPS        = gpsModule.getSpeed();
  data.NUM_SATS_GPS     = gpsModule.getSats();
  data.GPS_LAST         = millis();
  return true;
}

/*
 * Function: readPayload
 * -------------------
 * This function reads data from the Payload.
 */
bool Avionics::readPayload() {
  data.PAYLOAD_MESSAGE_SIZE = ValMU.getMessage();
  return true;
}

#ifdef HITL_ENABLED_FLAG
bool Avionics::simulateData() {
  DataFrame simulation = HITL.readData();
  data.LOOP_TIME                      = millis() - data.TIME;
  data.TIME                           = millis();
  data.RAW_PRESSURE_1                 = simulation.RAW_PRESSURE_1;
  data.RAW_PRESSURE_2                 = simulation.RAW_PRESSURE_2;
  data.RAW_PRESSURE_3                 = simulation.RAW_PRESSURE_3;
  data.RAW_PRESSURE_4                 = simulation.RAW_PRESSURE_4;
  data.BMP_1_ENABLE                   = simulation.BMP_1_ENABLE;
  data.BMP_2_ENABLE                   = simulation.BMP_2_ENABLE;
  data.BMP_3_ENABLE                   = simulation.BMP_3_ENABLE;
  data.BMP_4_ENABLE                   = simulation.BMP_4_ENABLE;
  // data.ALTITUDE                       = simulation.ALTITUDE;
  // data.ASCENT_RATE                    = simulation.ASCENT_RATE;
  data.PRESS_BASELINE                 = simulation.PRESS_BASELINE;
  // data.INCENTIVE_THRESHOLD            = simulation.INCENTIVE_THRESHOLD;
  // data.RE_ARM_CONSTANT                = simulation.RE_ARM_CONSTANT;
  data.BALLAST_ARM_ALT                = simulation.BALLAST_ARM_ALT;
  data.VALVE_SETPOINT                 = simulation.VALVE_SETPOINT;
  data.VALVE_VENT_DURATION            = simulation.VALVE_VENT_DURATION;
  // data.VALVE_ALT_LAST                 = simulation.VALVE_ALT_LAST;
  data.VALVE_VELOCITY_CONSTANT        = simulation.VALVE_VELOCITY_CONSTANT;
  // data.VALVE_ALTITUDE_DIFF_CONSTANT   = simulation.VALVE_ALTITUDE_DIFF_CONSTANT;
  // data.VALVE_LAST_ACTION_CONSTANT     = simulation.VALVE_LAST_ACTION_CONSTANT;
  data.BALLAST_SETPOINT               = simulation.BALLAST_SETPOINT;
  data.BALLAST_DROP_DURATION          = simulation.BALLAST_DROP_DURATION;
  // data.BALLAST_ALT_LAST               = simulation.BALLAST_ALT_LAST;
  data.BALLAST_VELOCITY_CONSTANT      = simulation.BALLAST_VELOCITY_CONSTANT;
  // data.BALLAST_ALTITUDE_DIFF_CONSTANT = simulation.BALLAST_ALTITUDE_DIFF_CONSTANT;
  // data.BALLAST_LAST_ACTION_CONSTANT   = simulation.BALLAST_LAST_ACTION_CONSTANT;
  // data.MANUAL_MODE                    = simulation.MANUAL_MODE;
  return true;
}
#endif

/*
 * Function: processData
 * -------------------
 * This function updates the current data frame with derived values.
 */
bool Avionics::processData() {
  bool success = true;
  filter.enableSensors(data.BMP_1_ENABLE, data.BMP_2_ENABLE, data.BMP_3_ENABLE, data.BMP_4_ENABLE);
  filter.storeData(data.TIME, data.RAW_PRESSURE_1, data.RAW_PRESSURE_2, data.RAW_PRESSURE_3, data.RAW_PRESSURE_4,data.PRESS_BASELINE);
  data.TEMP_INT                   = filter.getTemp(data.RAW_TEMP_1, data.RAW_TEMP_2, data.RAW_TEMP_3, data.RAW_TEMP_4);
  data.PRESS                      = filter.getPressure();
  data.BMP_1_REJECTIONS           = filter.getNumRejections(1);
  data.BMP_2_REJECTIONS           = filter.getNumRejections(2);
  data.BMP_3_REJECTIONS           = filter.getNumRejections(3);
  data.BMP_4_REJECTIONS           = filter.getNumRejections(4);

  data.CURRENT_TOTAL_AVG          = filter.getAvgCurrentSystem(data.CURRENT_TOTAL);
  data.CURRENT_TOTAL_MIN          = filter.getMinCurrentSystem();
  data.CURRENT_TOTAL_MAX          = filter.getMaxCurrentSystem();
  data.CURRENT_RB_AVG             = filter.getAvgCurrentRB(data.CURRENT_RB);
  data.CURRENT_RB_MAX             = filter.getMaxCurrentRB();
  data.CURRENT_MOTOR_VALVE_AVG    = filter.getAvgCurrentMotorValve(data.CURRENT_MOTOR_VALVE, (data.VALVE_STATE));
  data.CURRENT_MOTOR_VALVE_MAX    = filter.getMaxCurrentMotorValve();
  data.CURRENT_MOTOR_BALLAST_AVG  = filter.getAvgCurrentMotorBallast(data.CURRENT_MOTOR_BALLAST, (data.BALLAST_STATE));
  data.CURRENT_MOTOR_BALLAST_MAX  = filter.getMaxCurrentMotorBallast();
  data.CURRENT_PAYLOAD_AVG        = filter.getAvgCurrentPayload(data.CURRENT_PAYLOAD);
  data.CURRENT_PAYLOAD_MAX        = filter.getMaxCurrentPayload();

  data.LOOP_TIME_MAX              = _max(data.LOOP_TIME, data.LOOP_TIME_MAX);

  data.ALTITUDE_BAROMETER         = filter.getAltitude();
  data.ASCENT_RATE                = filter.getAscentRate();
  data.INCENTIVE_NOISE            = filter.getIncentiveNoise(data.BMP_1_ENABLE, data.BMP_2_ENABLE, data.BMP_3_ENABLE, data.BMP_4_ENABLE);
  if (data.ASCENT_RATE           >= 10) success = false;
  return success;
}

/*
 * Function: calcVitals
 * -------------------
 * This function calculates if the current state is within bounds.
 */
bool Avionics::calcVitals() {
  if(!data.SHOULD_REPORT) data.SHOULD_REPORT = (data.ASCENT_RATE >= 10);
  if(!data.MANUAL_MODE)   data.MANUAL_MODE   = (data.ASCENT_RATE >= 10);
  return true;
}

/*
 * Function: calcDebug
 * -------------------
 * This function calculates if the avionics is in debug mode.
 */
bool Avionics::calcDebug() {
  if(data.DEBUG_STATE && data.ALTITUDE_BAROMETER >= DEBUG_ALT) data.DEBUG_STATE = false;
  return true;
}

/*
 * Function: calcIncentives
 * -------------------
 * This function gets the updated incentives from the flight computer.
 */
bool Avionics::calcIncentives() {
  bool success = true;
  computer.updateValveConstants(data.VALVE_SETPOINT, data.VALVE_VELOCITY_CONSTANT, data.VALVE_ALTITUDE_DIFF_CONSTANT, data.VALVE_LAST_ACTION_CONSTANT);
  computer.updateBallastConstants(data.BALLAST_SETPOINT, data.BALLAST_VELOCITY_CONSTANT, data.BALLAST_ALTITUDE_DIFF_CONSTANT, data.BALLAST_LAST_ACTION_CONSTANT);
  data.RE_ARM_CONSTANT   = computer.updateControllerConstants(data.BALLAST_ARM_ALT, data.INCENTIVE_THRESHOLD);
  data.VALVE_ALT_LAST    = computer.getAltitudeSinceLastVentCorrected(data.ALTITUDE_BAROMETER, data.VALVE_ALT_LAST);
  data.BALLAST_ALT_LAST  = computer.getAltitudeSinceLastDropCorrected(data.ALTITUDE_BAROMETER, data.BALLAST_ALT_LAST);
  data.VALVE_INCENTIVE   = computer.getValveIncentive(data.ASCENT_RATE, data.ALTITUDE_BAROMETER, data.VALVE_ALT_LAST);
  data.BALLAST_INCENTIVE = computer.getBallastIncentive(data.ASCENT_RATE, data.ALTITUDE_BAROMETER, data.BALLAST_ALT_LAST);
  if (!data.MANUAL_MODE && data.VALVE_INCENTIVE >= 1 && data.BALLAST_INCENTIVE >= 1) success =  false;
  return success;
}

/*
 * Function: runHeaters
 * -------------------
 * This function thermally regulates the avionics. Disables heaters
 * if either the ballast or valve is running.
 */
bool Avionics::runHeaters() {
  if (!data.POWER_STATE_HEATER || data.VALVE_STATE || data.BALLAST_STATE) {
    PCB.turnOffHeaters();
  } else {
    PCB.heater(data.TEMP_SETPOINT, data.TEMP_INT, data.HEATER_STRONG_ENABLE, data.HEATER_WEEK_ENABLE);
  }
  return true;
}

/*
 * Function: runValve
 * -------------------
 * This function actuates the valve based on the calculated incentive.
 */
bool Avionics::runValve() {
  PCB.updateMechanicalConstants(data.VALVE_MOTOR_SPEED_OPEN, data.VALVE_MOTOR_SPEED_CLOSE, data.BALLAST_MOTOR_SPEED, data.VALVE_OPENING_DURATION, data.VALVE_CLOSING_DURATION);
  if((data.VALVE_INCENTIVE >= (1 + data.INCENTIVE_NOISE) && PCB.getValveQueue() <= QUEUE_APPEND_THRESHOLD) || data.FORCE_VALVE) {
    data.VALVE_NUM_ATTEMPTS++;
    bool shouldValve = (!data.MANUAL_MODE || data.FORCE_VALVE);
    if(shouldValve) data.VALVE_NUM_ACTIONS++;
    if(!data.FORCE_VALVE) data.VALVE_ALT_LAST = data.ALTITUDE_BAROMETER;
    uint32_t valveTime = data.VALVE_VENT_DURATION;
    if(data.FORCE_VALVE) valveTime = data.VALVE_FORCE_DURATION;
    if(shouldValve) data.VALVE_TIME_TOTAL += valveTime;
    PCB.EEPROMWritelong(EEPROM_VALVE_ALT_LAST, data.VALVE_ALT_LAST);
    PCB.queueValve(valveTime, shouldValve);
    data.FORCE_VALVE = false;
  }
  data.VALVE_QUEUE = PCB.getValveQueue();
  data.VALVE_STATE = PCB.checkValve(data.CURRENT_MOTOR_VALVE, data.VALVE_LEAK_INTERVAL);
  return true;
}

/*
 * Function: runBallast
 * -------------------
 * This function actuates the valve based on the calculated incentive.
 */
bool Avionics::runBallast() {
  PCB.updateMechanicalConstants(data.VALVE_MOTOR_SPEED_OPEN, data.VALVE_MOTOR_SPEED_CLOSE, data.BALLAST_MOTOR_SPEED, data.VALVE_OPENING_DURATION, data.VALVE_CLOSING_DURATION);
  if((data.BALLAST_INCENTIVE >= (1 + data.INCENTIVE_NOISE) && PCB.getBallastQueue() <= QUEUE_APPEND_THRESHOLD) || data.FORCE_BALLAST) {
    data.BALLAST_NUM_ATTEMPTS++;
    bool shouldBallast = (!data.MANUAL_MODE || data.FORCE_BALLAST);
    if(shouldBallast) data.BALLAST_NUM_ACTIONS++;
    if(!data.FORCE_BALLAST) data.BALLAST_ALT_LAST = data.ALTITUDE_BAROMETER;
    uint32_t ballastTime = data.BALLAST_DROP_DURATION;
    if(data.FORCE_BALLAST) ballastTime = data.BALLAST_FORCE_DURATION;
    if(shouldBallast) data.BALLAST_TIME_TOTAL += ballastTime;
    PCB.EEPROMWritelong(EEPROM_BALLAST_ALT_LAST, data.BALLAST_ALT_LAST);
    PCB.queueBallast(ballastTime, shouldBallast);
    data.FORCE_BALLAST = false;
  }
  data.BALLAST_QUEUE = PCB.getBallastQueue();
  data.BALLAST_NUM_OVERCURRENTS = PCB.getNumBallastOverCurrents();
  data.BALLAST_STATE = PCB.checkBallast(data.CURRENT_MOTOR_BALLAST, data.BALLAST_REVERSE_INTERVAL, data.BALLAST_STALL_CURRENT);
  return true;
}

/*
 * Function: runCutdown
 * -------------------
 * This function cuts down the payload if necessary.
 */
bool Avionics::runCutdown() {
  if(data.SHOULD_CUTDOWN) {
    PCB.cutDown();
    data.SHOULD_CUTDOWN = false;
    data.CUTDOWN_STATE = true;
    alert("completed cutdown", false);
  }
  return true;
}

/*
 * Function: runLED
 * -------------------
 * This function blinks the 1HZ LED required by the FAA.
 */
bool Avionics::runLED() {
  if (data.POWER_STATE_LED && (uint32_t(millis() / 1000.0) % 2 == 1)) PCB.runLED(true);
  else PCB.runLED(false);
  return true;
}

/*
 * Function: sendSATCOMS
 * -------------------
 * This function sends the current data frame over the ROCKBLOCK IO.
 */
bool Avionics::sendSATCOMS() {
  alert("sending Rockblock message", false);
  data.RB_SENT_COMMS++;
#ifndef RB_DISABLED_FLAG
  int16_t ret = RBModule.writeRead(COMMS_BUFFER, data.COMMS_LENGTH, data.RB_SHOULD_SLEEP);
  if(ret < 0) return false;
  clearVariables();
  if(ret > 0) parseCommand(ret);
#endif
  return true;
}

/*
 * Function: parseCommand
 * -------------------
 * This function parses the command received from the RockBLOCK.
 */
void Avionics::parseCommand(int16_t len) {
  COMMS_BUFFER[len] = 0;
  const char* commandStrFormat = "%d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s";
  uint8_t commandIndexes[8] = {0};
  char commandStrings[8][100] = {{0},{0},{0},{0},{0},{0},{0},{0}};
  uint8_t numScanned = sscanf(COMMS_BUFFER, commandStrFormat,
                              &commandIndexes[0], commandStrings[0],
                              &commandIndexes[1], commandStrings[1],
                              &commandIndexes[2], commandStrings[2],
                              &commandIndexes[3], commandStrings[3],
                              &commandIndexes[4], commandStrings[4],
                              &commandIndexes[5], commandStrings[5],
                              &commandIndexes[6], commandStrings[6],
                              &commandIndexes[7], commandStrings[7]);
  if (numScanned % 2 != 0) return;
  data.SHOULD_REPORT = true;

  for (uint8_t i = 0; i < numScanned / 2; i++) {
    uint8_t index = commandIndexes[i];
    if (index == CUTDOWN_INDEX && (strlen(CUTDOWN_COMMAND) == strlen(commandStrings[i])) && strncmp(commandStrings[i], CUTDOWN_COMMAND, strlen(commandStrings[i])) == 0) {
      data.SHOULD_CUTDOWN = true;
    }
    if (index < 0 || index > 80) return;
    char* charAfterNumbers;
    float commandValue = (float) strtod(commandStrings[i], &charAfterNumbers);
    if (*charAfterNumbers) return;
    updateConstant(index, commandValue);
  }
}

/*
 * Function: updateConstant
 * -------------------
 * This function updates the state appropriate state variable
 * based on the command index.
 */
void Avionics::updateConstant(uint8_t index, float value) {
  if      (index ==  0) data.VALVE_ALT_LAST = value;
  else if (index ==  1) data.BALLAST_ALT_LAST = value;
  else if (index ==  2) data.VALVE_SETPOINT = value;
  else if (index ==  3) data.BALLAST_SETPOINT = value;
  else if (index ==  4) data.BALLAST_ARM_ALT = value;
  else if (index ==  5) data.INCENTIVE_THRESHOLD = value;
  else if (index ==  6) data.VALVE_VELOCITY_CONSTANT = value;
  else if (index ==  7) data.VALVE_ALTITUDE_DIFF_CONSTANT = 1.0 / value;
  else if (index ==  8) data.VALVE_LAST_ACTION_CONSTANT = 1.0 / value;
  else if (index ==  9) data.BALLAST_VELOCITY_CONSTANT = value;
  else if (index == 10) data.BALLAST_ALTITUDE_DIFF_CONSTANT = 1.0 / value;
  else if (index == 11) data.BALLAST_LAST_ACTION_CONSTANT = 1.0 / value;
  else if (index == 12) data.VALVE_VENT_DURATION = value * 1000;
  else if (index == 13) data.BALLAST_DROP_DURATION = value * 1000;
  else if (index == 14) data.PRESS_BASELINE = value;
  else if (index == 15) data.BALLAST_REVERSE_INTERVAL = value * 1000;
  else if (index == 16) data.VALVE_LEAK_INTERVAL = value * 1000;
  else if (index == 17) data.BALLAST_STALL_CURRENT = value;
  else if (index == 18) data.VALVE_MOTOR_SPEED_OPEN = value;
  else if (index == 19) data.VALVE_MOTOR_SPEED_CLOSE = value;
  else if (index == 20) data.BALLAST_MOTOR_SPEED = value;
  else if (index == 21) data.VALVE_OPENING_DURATION = value * 1000;
  else if (index == 22) data.VALVE_CLOSING_DURATION = value * 1000;
  else if (index == 23) data.TEMP_SETPOINT = value;
  else if (index == 24) data.POWER_STATE_LED = value;
  else if (index == 25) data.RB_INTERVAL = value * 1000;
  else if (index == 26) data.GPS_INTERVAL = value * 1000;
  else if (index == 27) parseManualCommand(value);
  else if (index == 28) parseReportCommand(value);
  else if (index == 29) parseSensorsCommand(value);
  else if (index == 30) parseValveCommand(value * 1000);
  else if (index == 31) parseBallastCommand(value * 1000);
  else if (index == 32) parseRockBLOCKPowerCommand(value);
  else if (index == 33) parseRockBLOCKModeCommand(value);
  else if (index == 34) parseGPSPowerCommand(value);
  else if (index == 35) parseHeaterPowerCommand(value);
  else if (index == 36) parseHeaterModeCommand(value);
  else if (index == 37) parsePayloadPowerCommand(value);
}

/*
 * Function: parseManualCommand
 * -------------------
 * This function parses the manual mode command.
 */
void Avionics::parseManualCommand(bool command) {
  PCB.clearValveQueue();
  PCB.clearBallastQueue();
  data.MANUAL_MODE = command;
}

/*
 * Function: parseReportCommand
 * -------------------
 * This function parses the REPORT_MODE mode command.
 */
void Avionics::parseReportCommand(uint8_t command) {
  data.REPORT_MODE = command;
}

/*
 * Function: parseSensorsCommand
 * -------------------
 * This function parses the active sensors.
 */
void Avionics::parseSensorsCommand(uint8_t command) {
  data.BMP_1_ENABLE = command & 0b0001;
  data.BMP_2_ENABLE = command & 0b0010;
  data.BMP_3_ENABLE = command & 0b0100;
  data.BMP_4_ENABLE = command & 0b1000;
}

/*
 * Function: parseValveCommand
 * -------------------
 * This function parses a forced valve command.
 */
void Avionics::parseValveCommand(uint32_t command) {
  if(command == 0) PCB.clearValveQueue();
  else {
    data.FORCE_VALVE = true;
    data.VALVE_FORCE_DURATION = command;
  }
}

/*
 * Function: parseBallastCommand
 * -------------------
 * This function parses a forced ballast command.
 */
void Avionics::parseBallastCommand(uint32_t command) {
  if(command == 0) PCB.clearBallastQueue();
  else {
    data.FORCE_BALLAST = true;
    data.BALLAST_FORCE_DURATION = command;
  }
}

/*
 * Function: parseRockBLOCKPowerCommand
 * -------------------
 * This function parses the power RockBLOCK command.
 */
void Avionics::parseRockBLOCKPowerCommand(bool command) {
  if (command && !data.POWER_STATE_RB) {
    data.POWER_STATE_RB = true;
    RBModule.restart(data.RB_SHOULD_SLEEP);
  }
  else if (!command) {
    data.POWER_STATE_RB = false;
    RBModule.shutdown();
  }
  data.RB_LAST = millis();
}

/*
 * Function: parseRockBLOCKModeCommand
 * -------------------
 * This function parses the mode RockBLOCK command.
 */
void Avionics::parseRockBLOCKModeCommand(bool command) {
  if(data.RB_SHOULD_SLEEP && !command) RBModule.wake();
  if(!data.RB_SHOULD_SLEEP && command) RBModule.snooze();
  data.RB_SHOULD_SLEEP = command;
  data.RB_LAST = millis();
}

/*
 * Function: parseGPSPowerCommand
 * -------------------
 * This function parses the GPS power command.
 */
void Avionics::parseGPSPowerCommand(uint8_t command) {
  if (command == 0) {
    data.POWER_STATE_GPS = false;
    gpsModule.shutdown();
  }
  else if (command == 1) {
    data.POWER_STATE_GPS = true;
    gpsModule.restart();
  }
  else if (command == 2) {
    gpsModule.hotstart();
    readGPS();
  }
}

/*
 * Function: parseHeaterPowerCommand
 * -------------------
 * This function parses the heater power command.
 */
void Avionics::parseHeaterPowerCommand(bool command) {
  data.POWER_STATE_HEATER = command;
  PCB.setHeaterMode(command);
  if (!command) PCB.turnOffHeaters();
}

/*
 * Function: parseHeaterModeCommand
 * -------------------
 * This function parses the heater mode.
 */
void Avionics::parseHeaterModeCommand(uint8_t command) {
  data.HEATER_STRONG_ENABLE = command & 0b0001;
  data.HEATER_WEEK_ENABLE   = command & 0b0010;
}

/*
 * Function: parsePayloadPowerCommand
 * -------------------
 * This function parses the Payload power command.
 */
void Avionics::parsePayloadPowerCommand(bool command) {
  if (command && !data.POWER_STATE_PAYLOAD) {
    data.POWER_STATE_PAYLOAD = true;
    ValMU.restart();
  }
  else if (!command) {
    data.POWER_STATE_PAYLOAD = false;
    ValMU.shutdown();
  }
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
 * Function: printHeader
 * -------------------
 * This function prints the CSV header.
 */
void Avionics::printHeader() {
  Serial.print("Stanford Student Space Initiative Balloons Launch ");
  Serial.print(MISSION_NUMBER);
  Serial.print('\n');
}

/*
 * Function: alert
 * -------------------
 * This function alerts important information whenever a specific event occurs.
 */
void Avionics::alert(const char* debug, bool fatal) {
  if(!data.DEBUG_STATE) return;
  Serial.print(millis());
  Serial.print(',');
  if(fatal) Serial.print("FATAL ERROR!!!!!!!!!!: ");
  else Serial.print("Alert: ");
  Serial.print(debug);
  Serial.print("...\n");
}

/*
 * Function: clearVariables
 * -------------------
 * This function clears the interval based variables at the end of
 * a successfull transmission.
 */
void Avionics::clearVariables() {
  filter.clearCurrentValues();
  PCB.clearBallastOverCurrents();
  data.VALVE_NUM_ACTIONS = 0;
  data.BALLAST_NUM_ACTIONS = 0;
  data.VALVE_NUM_ATTEMPTS = 0;
  data.BALLAST_NUM_ATTEMPTS = 0;
  data.LOOP_TIME_MAX = 0;
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
    if (bitIndex < 0) {
      bitIndex = 7;
      byteIndex++;
    }
  }
  return resolution;
}

/*
 * Function: compressData
 * -------------------
 * This function compresses the data frame into a bit stream.
 * The total bitstream cannot exceed 400 bytes.
 */
int16_t Avionics::compressData() {
  int16_t lengthBits  = 0;
  int16_t lengthBytes = 0;
  for(uint16_t i = 0; i < COMMS_BUFFER_SIZE; i++) COMMS_BUFFER[i] = 0;
  lengthBits += compressVariable(data.TIME / 1000,                           0,    3000000, 20, lengthBits); // time
  lengthBits += compressVariable(data.LAT_GPS,                              -90,   90,      21, lengthBits); // latitude
  lengthBits += compressVariable(data.LONG_GPS,                             -180,  180,     22, lengthBits); // longitude
  lengthBits += compressVariable(data.ALTITUDE_BAROMETER,                   -2000, 40000,   16, lengthBits); // altitude_barometer
  lengthBits += compressVariable(data.ALTITUDE_GPS,                         -2000, 40000,   14, lengthBits);
  lengthBits += compressVariable(data.ASCENT_RATE,                          -10,   10,      11, lengthBits);
  lengthBits += compressVariable(data.VALVE_INCENTIVE,                      -50,   10,      12, lengthBits);
  lengthBits += compressVariable(data.BALLAST_INCENTIVE,                    -50,   10,      12, lengthBits);
  lengthBits += compressVariable(data.VALVE_STATE,                           0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_STATE,                         0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.VALVE_QUEUE / 1000,                    0,    1023,    10, lengthBits);
  lengthBits += compressVariable(data.BALLAST_QUEUE / 1000,                  0,    1023,    10, lengthBits);
  lengthBits += compressVariable(data.VALVE_TIME_TOTAL / 1000,               0,    16383,   13, lengthBits); // valve time total
  lengthBits += compressVariable(data.BALLAST_TIME_TOTAL / 1000,             0,    16383,   13, lengthBits); // ballast time total
  lengthBits += compressVariable(data.VALVE_NUM_ACTIONS,                     0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_NUM_ACTIONS,                   0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.VALVE_NUM_ATTEMPTS,                    0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_NUM_ATTEMPTS,                  0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_NUM_OVERCURRENTS,              0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.CUTDOWN_STATE,                         0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.TEMP_INT,                             -85,   65,      9,  lengthBits);
  lengthBits += compressVariable(data.JOULES_TOTAL,                          0,    1572863, 18, lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_PRIMARY,                       0,    6,       9,  lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_5V,                            4,    6,       7,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_TOTAL_AVG,                     0,    4095,    12, lengthBits);
  lengthBits += compressVariable(data.CURRENT_TOTAL_MIN,                     0,    4095,    12, lengthBits);
  lengthBits += compressVariable(data.CURRENT_TOTAL_MAX,                     0,    4095,    12, lengthBits);
  lengthBits += compressVariable(data.CURRENT_RB_AVG,                        0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_RB_MAX,                        0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_VALVE_AVG,               0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_VALVE_MAX,               0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_BALLAST_AVG,             0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_BALLAST_MAX,             0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_PAYLOAD_AVG,                   0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_PAYLOAD_MAX,                   0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.LOOP_TIME_MAX,                         0,    10239,   10, lengthBits);
  lengthBits += compressVariable(data.RB_SENT_COMMS,                         0,    8191,    13, lengthBits);
  lengthBits += compressVariable(data.RB_SLEEP_FAILS,                        0,    8191,    13, lengthBits);
  lengthBits += compressVariable(data.MANUAL_MODE,                           0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.REPORT_MODE,                           0,    2,       2,  lengthBits);
  lengthBits += compressVariable(data.SHOULD_REPORT,                         0,    1,       1,  lengthBits);
  if (data.SHOULD_REPORT || data.REPORT_MODE != 0) {
    lengthBits += compressVariable(data.POWER_STATE_LED,                     0,    1,       1,  lengthBits); // LED Power state
    lengthBits += compressVariable(data.POWER_STATE_RB,                      0,    1,       1,  lengthBits); // RB Power State
    lengthBits += compressVariable(data.POWER_STATE_GPS,                     0,    1,       1,  lengthBits); // GPS Power State
    lengthBits += compressVariable(data.POWER_STATE_HEATER,                  0,    1,       1,  lengthBits); // Heater Power State
    lengthBits += compressVariable(data.POWER_STATE_PAYLOAD,                 0,    1,       1,  lengthBits); // Payload Power State
    lengthBits += compressVariable(data.HEATER_STRONG_ENABLE,                0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.HEATER_WEEK_ENABLE,                  0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.NUM_SATS_GPS,                        0,    15,      3,  lengthBits);
    lengthBits += compressVariable(data.INCENTIVE_NOISE,                     0,    4,       8,  lengthBits);
    lengthBits += compressVariable(data.RE_ARM_CONSTANT,                     0,    4,       8,  lengthBits);
    lengthBits += compressVariable(data.VALVE_ALT_LAST,                     -2000, 50000,   11, lengthBits); // Altitude During Last Venting Event
    lengthBits += compressVariable(data.BALLAST_ALT_LAST,                   -2000, 50000,   11, lengthBits); // Altitude During Last Ballast Event
    lengthBits += compressVariable(data.DEBUG_STATE,                         0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.FORCE_VALVE,                         0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.FORCE_BALLAST,                       0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_1_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_2_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_3_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_4_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(log2(data.BMP_1_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_1_logrejections
    lengthBits += compressVariable(log2(data.BMP_2_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_2_logrejections
    lengthBits += compressVariable(log2(data.BMP_3_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_3_logrejections
    lengthBits += compressVariable(log2(data.BMP_4_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_4_logrejections
    lengthBits += compressVariable(data.JOULES_HEATER,                       0,    819199,  13, lengthBits);
  }
  if (data.SHOULD_REPORT || data.REPORT_MODE == 2) {
    lengthBits += compressVariable(data.TEMP_SETPOINT,                      -70,   40,      6,  lengthBits); // Payload temperature setpoint
    lengthBits += compressVariable(data.RB_INTERVAL / 1000,                  0,    1023,    10, lengthBits); // RB communication interval
    lengthBits += compressVariable(data.GPS_INTERVAL / 1000,                 0,    1023,    10, lengthBits); // GPS communication interval
    lengthBits += compressVariable(data.RB_SHOULD_SLEEP,                     0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.PRESS_BASELINE,                      0,    131071,  17, lengthBits); // Pressure baseline
    lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,                 0,    4,       3,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_ARM_ALT,                    -2000, 40000,   16, lengthBits); // Ballast Arming Altitude
    lengthBits += compressVariable(data.BALLAST_REVERSE_INTERVAL / 1000,     0,    1599,    4,  lengthBits); // Ballast reverse interval
    lengthBits += compressVariable(data.VALVE_LEAK_INTERVAL / 1000,          0,    1599,    4,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_STALL_CURRENT,               0,    511,     4,  lengthBits);
    lengthBits += compressVariable(data.VALVE_OPENING_DURATION / 1000,       0,    10,      5,  lengthBits);
    lengthBits += compressVariable(data.VALVE_CLOSING_DURATION / 1000,       0,    10,      5,  lengthBits);
    lengthBits += compressVariable(data.VALVE_SETPOINT,                     -2000, 50000,   11, lengthBits);
    lengthBits += compressVariable(data.VALVE_VENT_DURATION / 1000,          0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.VALVE_FORCE_DURATION / 1000,         0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.VALVE_VELOCITY_CONSTANT,             0,    5,       8,  lengthBits); // Valve Speed Constant
    lengthBits += compressVariable(1.0 / data.VALVE_ALTITUDE_DIFF_CONSTANT,  0,    4095,    8,  lengthBits); // Valve Altitude Difference Constant
    lengthBits += compressVariable(1.0 / data.VALVE_LAST_ACTION_CONSTANT,    0,    4095,    8,  lengthBits); // Valve last action constant
    lengthBits += compressVariable(data.BALLAST_SETPOINT,                   -2000, 50000,   11, lengthBits);
    lengthBits += compressVariable(data.BALLAST_DROP_DURATION / 1000,        0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_FORCE_DURATION / 1000,       0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_VELOCITY_CONSTANT,           0,    5,       8,  lengthBits); // Ballast Speed Constant
    lengthBits += compressVariable(1.0 / data.BALLAST_ALTITUDE_DIFF_CONSTANT,0,    4095,    8,  lengthBits); // Ballast Altitude Difference Constant
    lengthBits += compressVariable(1.0 / data.BALLAST_LAST_ACTION_CONSTANT,  0,    4095,    8,  lengthBits); // Ballast last action constant
  }
  lengthBits += 8 - (lengthBits % 8);
  lengthBytes = lengthBits / 8;
  for (size_t i = 0; i < data.PAYLOAD_MESSAGE_SIZE; i++) {
    COMMS_BUFFER[lengthBytes + i] = ValMU.buf[i];
  }
  lengthBytes += data.PAYLOAD_MESSAGE_SIZE;
  data.SHOULD_REPORT = false;
  data.COMMS_LENGTH = lengthBytes;
  if(data.DEBUG_STATE) {
    for (int16_t i = 0; i < lengthBytes; i++) {
      uint8_t byte = COMMS_BUFFER[i];
      (byte & 0x80 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x40 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x20 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x10 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x08 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x04 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x02 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x01 ? Serial.print('1') : Serial.print('0'));
    }
    Serial.print('\n');
  }
  return lengthBytes;
}

/*
 * Function: printState
 * -------------------
 * This function prints the current avionics state.
 */
void Avionics::printState() {
  Serial.print("TIME:");
  Serial.print(data.TIME);
  Serial.print(',');
  Serial.print(" LAT_GPS:");
  Serial.print(data.LAT_GPS, 4);
  Serial.print(',');
  Serial.print(" LONG_GPS:");
  Serial.print(data.LONG_GPS, 4);
  Serial.print(',');
  Serial.print(" ALTITUDE_BAROMETER:");
  Serial.print(data.ALTITUDE_BAROMETER);
  Serial.print(',');
  Serial.print(" ALTITUDE_GPS:");
  Serial.print(data.ALTITUDE_GPS);
  Serial.print(',');
  Serial.print(" ASCENT_RATE:");
  Serial.print(data.ASCENT_RATE);
  Serial.print(',');
  Serial.print(" VALVE_INCENTIVE:");
  Serial.print(data.VALVE_INCENTIVE);
  Serial.print(',');
  Serial.print(" BALLAST_INCENTIVE:");
  Serial.print(data.BALLAST_INCENTIVE);
  Serial.print(',');
  Serial.print(" VALVE_STATE:");
  Serial.print(data.VALVE_STATE);
  Serial.print(',');
  Serial.print(" BALLAST_STATE:");
  Serial.print(data.BALLAST_STATE);
  Serial.print(',');
  Serial.print(" VALVE_QUEUE:");
  Serial.print(data.VALVE_QUEUE);
  Serial.print(',');
  Serial.print(" BALLAST_QUEUE:");
  Serial.print(data.BALLAST_QUEUE);
  Serial.print(',');
  Serial.print(" VALVE_TIME_TOTAL:");
  Serial.print(data.VALVE_TIME_TOTAL);
  Serial.print(',');
  Serial.print(" BALLAST_TIME_TOTAL:");
  Serial.print(data.BALLAST_TIME_TOTAL);
  Serial.print(',');
  Serial.print(" VALVE_NUM_ACTIONS:");
  Serial.print(data.VALVE_NUM_ACTIONS);
  Serial.print(',');
  Serial.print(" BALLAST_NUM_ACTIONS:");
  Serial.print(data.BALLAST_NUM_ACTIONS);
  Serial.print(',');
  Serial.print(" VALVE_NUM_ATTEMPTS:");
  Serial.print(data.VALVE_NUM_ATTEMPTS);
  Serial.print(',');
  Serial.print(" BALLAST_NUM_ATTEMPTS:");
  Serial.print(data.BALLAST_NUM_ATTEMPTS);
  Serial.print(',');
  Serial.print(" BALLAST_NUM_OVERCURRENTS:");
  Serial.print(data.BALLAST_NUM_OVERCURRENTS);
  Serial.print(',');
  Serial.print(" CUTDOWN_STATE:");
  Serial.print(data.CUTDOWN_STATE);
  Serial.print(',');
  Serial.print(" TEMP_INT:");
  Serial.print(data.TEMP_INT);
  Serial.print(',');
  Serial.print(" JOULES_TOTAL:");
  Serial.print(data.JOULES_TOTAL);
  Serial.print(',');
  Serial.print(" VOLTAGE_PRIMARY:");
  Serial.print(data.VOLTAGE_PRIMARY);
  Serial.print(',');
  Serial.print(" VOLTAGE_5V:");
  Serial.print(data.VOLTAGE_5V);
  Serial.print(',');
  Serial.print(" CURRENT_TOTAL_AVG:");
  Serial.print(data.CURRENT_TOTAL_AVG);
  Serial.print(',');
  Serial.print(" CURRENT_TOTAL_MIN:");
  Serial.print(data.CURRENT_TOTAL_MIN);
  Serial.print(',');
  Serial.print(" CURRENT_TOTAL_MAX:");
  Serial.print(data.CURRENT_TOTAL_MAX);
  Serial.print(',');
  Serial.print(" CURRENT_RB_AVG:");
  Serial.print(data.CURRENT_RB_AVG);
  Serial.print(',');
  Serial.print(" CURRENT_RB_MAX:");
  Serial.print(data.CURRENT_RB_MAX);
  Serial.print(',');
  Serial.print(" CURRENT_MOTOR_VALVE_AVG:");
  Serial.print(data.CURRENT_MOTOR_VALVE_AVG);
  Serial.print(',');
  Serial.print(" CURRENT_MOTOR_VALVE_MAX:");
  Serial.print(data.CURRENT_MOTOR_VALVE_MAX);
  Serial.print(',');
  Serial.print(" CURRENT_MOTOR_BALLAST_AVG:");
  Serial.print(data.CURRENT_MOTOR_BALLAST_AVG);
  Serial.print(',');
  Serial.print(" CURRENT_MOTOR_BALLAST_MAX:");
  Serial.print(data.CURRENT_MOTOR_BALLAST_MAX);
  Serial.print(',');
  Serial.print(" CURRENT_PAYLOAD_AVG:");
  Serial.print(data.CURRENT_PAYLOAD_AVG);
  Serial.print(',');
  Serial.print(" CURRENT_PAYLOAD_MAX:");
  Serial.print(data.CURRENT_PAYLOAD_MAX);
  Serial.print(',');
  Serial.print(" LOOP_TIME_MAX:");
  Serial.print(data.LOOP_TIME_MAX);
  Serial.print(',');
  Serial.print(" LOOP_TIME_MAX:");
  Serial.print(data.LOOP_TIME_MAX);
  Serial.print(',');
  Serial.print(" RB_SENT_COMMS:");
  Serial.print(data.RB_SENT_COMMS);
  Serial.print(',');
  Serial.print(" RB_WAKE_FAILS:");
  Serial.print(data.RB_WAKE_FAILS);
  Serial.print(',');
  Serial.print(" RB_SLEEP_FAILS:");
  Serial.print(data.RB_SLEEP_FAILS);
  Serial.print(',');
  Serial.print(" MANUAL_MODE:");
  Serial.print(data.MANUAL_MODE);
  Serial.print(',');
  Serial.print(" REPORT_MODE:");
  Serial.print(data.REPORT_MODE);
  Serial.print(',');
  Serial.print(" SHOULD_REPORT:");
  Serial.print(data.SHOULD_REPORT);
  Serial.print(',');
  Serial.print(" POWER_STATE_LED:");
  Serial.print(data.POWER_STATE_LED);
  Serial.print(',');
  Serial.print(" POWER_STATE_RB:");
  Serial.print(data.POWER_STATE_RB);
  Serial.print(',');
  Serial.print(" POWER_STATE_GPS:");
  Serial.print(data.POWER_STATE_GPS);
  Serial.print(',');
  Serial.print(" POWER_STATE_HEATER:");
  Serial.print(data.POWER_STATE_HEATER);
  Serial.print(',');
  Serial.print(" POWER_STATE_PAYLOAD:");
  Serial.print(data.POWER_STATE_PAYLOAD);
  Serial.print(',');
  Serial.print(" HEATER_STRONG_ENABLE:");
  Serial.print(data.HEATER_STRONG_ENABLE);
  Serial.print(',');
  Serial.print(" HEATER_WEEK_ENABLE:");
  Serial.print(data.HEATER_WEEK_ENABLE);
  Serial.print(',');
  Serial.print(" NUM_SATS_GPS:");
  Serial.print(data.NUM_SATS_GPS);
  Serial.print(',');
  Serial.print(" SPEED_GPS:");
  Serial.print(data.SPEED_GPS);
  Serial.print(',');
  Serial.print(" HEADING_GPS:");
  Serial.print(data.HEADING_GPS);
  Serial.print(',');
  Serial.print(" INCENTIVE_NOISE:");
  Serial.print(data.INCENTIVE_NOISE);
  Serial.print(',');
  Serial.print(" RE_ARM_CONSTANT:");
  Serial.print(data.RE_ARM_CONSTANT);
  Serial.print(',');
  Serial.print(" VALVE_ALT_LAST:");
  Serial.print(data.VALVE_ALT_LAST);
  Serial.print(',');
  Serial.print(" BALLAST_ALT_LAST:");
  Serial.print(data.BALLAST_ALT_LAST);
  Serial.print(',');
  Serial.print(" DEBUG_STATE:");
  Serial.print(data.DEBUG_STATE);
  Serial.print(',');
  Serial.print(" FORCE_VALVE:");
  Serial.print(data.FORCE_VALVE);
  Serial.print(',');
  Serial.print(" FORCE_BALLAST:");
  Serial.print(data.FORCE_BALLAST);
  Serial.print(',');
  Serial.print(" BMP_1_ENABLE:");
  Serial.print(data.BMP_1_ENABLE);
  Serial.print(',');
  Serial.print(" BMP_2_ENABLE:");
  Serial.print(data.BMP_2_ENABLE);
  Serial.print(',');
  Serial.print(" BMP_3_ENABLE:");
  Serial.print(data.BMP_3_ENABLE);
  Serial.print(',');
  Serial.print(" BMP_4_ENABLE:");
  Serial.print(data.BMP_4_ENABLE);
  Serial.print(',');
  Serial.print(" BMP_1_REJECTIONS:");
  Serial.print(data.BMP_1_REJECTIONS);
  Serial.print(',');
  Serial.print(" BMP_2_REJECTIONS:");
  Serial.print(data.BMP_2_REJECTIONS);
  Serial.print(',');
  Serial.print(" BMP_3_REJECTIONS:");
  Serial.print(data.BMP_3_REJECTIONS);
  Serial.print(',');
  Serial.print(" BMP_4_REJECTIONS:");
  Serial.print(data.BMP_4_REJECTIONS);
  Serial.print(',');
  Serial.print(',');
  Serial.print(" JOULES_HEATER:");
  Serial.print(data.JOULES_HEATER);
  Serial.print(',');
  Serial.print(" TEMP_SETPOINT:");
  Serial.print(data.TEMP_SETPOINT);
  Serial.print(',');
  Serial.print(" RB_INTERVAL:");
  Serial.print(data.RB_INTERVAL);
  Serial.print(',');
  Serial.print(" GPS_INTERVAL:");
  Serial.print(data.GPS_INTERVAL);
  Serial.print(',');
  Serial.print(" RB_SHOULD_SLEEP:");
  Serial.print(data.RB_SHOULD_SLEEP);
  Serial.print(',');
  Serial.print(" PRESS_BASELINE:");
  Serial.print(data.PRESS_BASELINE);
  Serial.print(',');
  Serial.print(" INCENTIVE_THRESHOLD:");
  Serial.print(data.INCENTIVE_THRESHOLD);
  Serial.print(',');
  Serial.print(" BALLAST_ARM_ALT:");
  Serial.print(data.BALLAST_ARM_ALT);
  Serial.print(',');
  Serial.print(" BALLAST_REVERSE_INTERVAL:");
  Serial.print(data.BALLAST_REVERSE_INTERVAL);
  Serial.print(',');
  Serial.print(" VALVE_LEAK_INTERVAL:");
  Serial.print(data.VALVE_LEAK_INTERVAL);
  Serial.print(',');
  Serial.print(" BALLAST_STALL_CURRENT:");
  Serial.print(data.BALLAST_STALL_CURRENT);
  Serial.print(',');
  Serial.print(" VALVE_MOTOR_SPEED_OPEN:");
  Serial.print(data.VALVE_MOTOR_SPEED_OPEN);
  Serial.print(',');
  Serial.print(" VALVE_MOTOR_SPEED_CLOSE:");
  Serial.print(data.VALVE_MOTOR_SPEED_CLOSE);
  Serial.print(',');
  Serial.print(" BALLAST_MOTOR_SPEED:");
  Serial.print(data.BALLAST_MOTOR_SPEED);
  Serial.print(',');
  Serial.print(" VALVE_OPENING_DURATION:");
  Serial.print(data.VALVE_OPENING_DURATION);
  Serial.print(',');
  Serial.print(" VALVE_CLOSING_DURATION:");
  Serial.print(data.VALVE_CLOSING_DURATION);
  Serial.print(',');
  Serial.print(" VALVE_SETPOINT:");
  Serial.print(data.VALVE_SETPOINT);
  Serial.print(',');
  Serial.print(" VALVE_VENT_DURATION:");
  Serial.print(data.VALVE_VENT_DURATION);
  Serial.print(',');
  Serial.print(" VALVE_FORCE_DURATION:");
  Serial.print(data.VALVE_FORCE_DURATION);
  Serial.print(',');
  Serial.print(" VALVE_VELOCITY_CONSTANT:");
  Serial.print(data.VALVE_VELOCITY_CONSTANT);
  Serial.print(',');
  Serial.print(" VALVE_ALTITUDE_DIFF_CONSTANT (1.0 / ):");
  Serial.print(1.0 / data.VALVE_ALTITUDE_DIFF_CONSTANT);
  Serial.print(',');
  Serial.print(" VALVE_LAST_ACTION_CONSTANT (1.0 / ):");
  Serial.print(1.0 / data.VALVE_LAST_ACTION_CONSTANT);
  Serial.print(',');
  Serial.print(" BALLAST_SETPOINT:");
  Serial.print(data.BALLAST_SETPOINT);
  Serial.print(',');
  Serial.print(" BALLAST_DROP_DURATION:");
  Serial.print(data.BALLAST_DROP_DURATION);
  Serial.print(',');
  Serial.print(" BALLAST_FORCE_DURATION:");
  Serial.print(data.BALLAST_FORCE_DURATION);
  Serial.print(',');
  Serial.print(" BALLAST_VELOCITY_CONSTANT:");
  Serial.print(data.BALLAST_VELOCITY_CONSTANT);
  Serial.print(',');
  Serial.print(" BALLAST_ALTITUDE_DIFF_CONSTANT (1.0 / ):");
  Serial.print(1.0 / data.BALLAST_ALTITUDE_DIFF_CONSTANT);
  Serial.print(',');
  Serial.print(" BALLAST_LAST_ACTION_CONSTANT (1.0 / ):");
  Serial.print(1.0 / data.BALLAST_LAST_ACTION_CONSTANT);
  Serial.print(',');
  Serial.print(" SETUP_STATE:");
  Serial.print(data.SETUP_STATE);
  Serial.print(',');
  Serial.print(" SHOULD_CUTDOWN:");
  Serial.print(data.SHOULD_CUTDOWN);
  Serial.print(',');
  Serial.print(" LOOP_TIME:");
  Serial.print(data.LOOP_TIME);
  Serial.print(',');
  Serial.print(" RAW_TEMP_1:");
  Serial.print(data.RAW_TEMP_1);
  Serial.print(',');
  Serial.print(" RAW_TEMP_2:");
  Serial.print(data.RAW_TEMP_2);
  Serial.print(',');
  Serial.print(" RAW_TEMP_3:");
  Serial.print(data.RAW_TEMP_3);
  Serial.print(',');
  Serial.print(" RAW_TEMP_4:");
  Serial.print(data.RAW_TEMP_4);
  Serial.print(',');
  Serial.print(" RAW_PRESSURE_1:");
  Serial.print(data.RAW_PRESSURE_1);
  Serial.print(',');
  Serial.print(" RAW_PRESSURE_2:");
  Serial.print(data.RAW_PRESSURE_2);
  Serial.print(',');
  Serial.print(" RAW_PRESSURE_3:");
  Serial.print(data.RAW_PRESSURE_3);
  Serial.print(',');
  Serial.print(" RAW_PRESSURE_4:");
  Serial.print(data.RAW_PRESSURE_4);
  Serial.print(',');
  Serial.print(" PRESS:");
  Serial.print(data.PRESS);
  Serial.print(',');
  Serial.print(" CURRENT_USB:");
  Serial.print(data.CURRENT_USB);
  Serial.print(',');
  Serial.print(" CURRENT_TOTAL:");
  Serial.print(data.CURRENT_TOTAL);
  Serial.print(',');
  Serial.print(" CURRENT_RB:");
  Serial.print(data.CURRENT_RB);
  Serial.print(',');
  Serial.print(" CURRENT_MOTOR_VALVE:");
  Serial.print(data.CURRENT_MOTOR_VALVE);
  Serial.print(',');
  Serial.print(" CURRENT_MOTOR_BALLAST:");
  Serial.print(data.CURRENT_MOTOR_BALLAST);
  Serial.print(',');
  Serial.print(" CURRENT_PAYLOAD:");
  Serial.print(data.CURRENT_PAYLOAD);
  Serial.print(',');
  Serial.print(" PAYLOAD_MESSAGE_SIZE:");
  Serial.print(data.PAYLOAD_MESSAGE_SIZE);
  Serial.print(',');
  Serial.print(" PID_OUTPUT:");
  Serial.print(data.PID_OUTPUT);
  Serial.print(',');
  Serial.print(" GPS_LAST:");
  Serial.print(data.GPS_LAST);
  Serial.print(',');
  Serial.print(" RB_LAST:");
  Serial.print(data.RB_LAST);
  Serial.print(',');
  Serial.print(" DATAFILE_LAST:");
  Serial.print(data.DATAFILE_LAST);
  Serial.print(',');
  Serial.print(" COMMS_LENGTH:");
  Serial.print(data.COMMS_LENGTH);
  Serial.print("\n\r");
  Serial.print("\n\r");
}
