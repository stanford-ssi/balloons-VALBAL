/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  John Dean | deanjl@stanford.edu
  Ben Newman | blnewman@stanford.edu
  Keegan Mehall | kmehall@stanford.edu
  Jonathan Zwiebel | jzwiebel@stanford.edu

  File: Avionics.cpp
  --------------------------
  Implementation of Avionics.h
*/

#include "Avionics.h"

//#define Serial Serial2

IntervalTimer sixtyScoreRevolutionsPerMinute;

volatile int numExecutions = 0;

void rpmCounter() {
  numExecutions++;
}

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the avionics flight controller.
 */
void Avionics::init() {
  #ifdef JANKSHITL
  //stepsim.setSS(101000);
  //tempsim.setSS(0);
  #endif
  Serial.begin(CONSOLE_BAUD);
  delay(3000);
  PCB.init();
  actuator.init();
  delay(500);
  Serial.println("setting payload high");


  // here be heaters
  pinMode(36, OUTPUT);
  pinMode(OP_PIN, INPUT);
  pinMode(VR_PIN, INPUT);

  pinMode(57, OUTPUT);
  digitalWrite(57, LOW);

  //if(!setupSDCard())                          alert("unable to initialize SD Card", true);
  if(!readHistory())                          alert("unable to initialize EEPROM", true);
  if(!sensors.init())                         alert("unable to initialize Sensors", true);
  delay(2000);
  Serial.println("Serial has been init");
  if(!currentSensor.init(CURRENT_MONITOR_CS)) alert("unable to initialize Current Sensor", true);
// #ifdef HITL_ENABLED_FLAG
//   if(!HITL.init())                            alert("unable to initialize Simulations", true);
// #endif
  if(!filter.init())                          alert("unable to initialize Filters", true);
  if(!computer.init())                        alert("unable to initialize Flight Controller", true) ;
  if(!gpsModule.init(data.POWER_STATE_GPS))   alert("unable to initialize GPS", true);
  if(!superCap.init())                        alert("unable to initialize superCap", true);
  if(!setup5VLine())                          alert("unable to initialize 5V line", true);
  // pinMode(49, OUTPUT);
  // digitalWrite(49, HIGH);
  // pinMode(56, OUTPUT);
  // digitalWrite(56, HIGH);
#ifndef RB_DISABLED_FLAG
  if(!RBModule.init(false))     alert("unable to initialize RockBlock", true);
#endif
  if(!radio.init(data.POWER_STATE_RADIO)) alert("unable to initialize Payload", true);
  data.TIME = millis();
  data.SETUP_STATE = false;

  #ifdef SERIALSHITL
  //holds until connection with PC is established
  while(true){
    Serial.write(FSTART);
    delay(100);
    if(Serial.available()){
      if(Serial.read() == FSTART){
        break;
      }
    }
  }
  #endif

  sixtyScoreRevolutionsPerMinute.begin(rpmCounter, 50000);
}

/*
 * Function: test
 * -------------------
 * This function tests the hardware.
 */
void Avionics::test() {
  alert("Initializing test...", true);

    actuator.queueBallast(5000, true);
      actuator.queueValve(5000, true);
  //actuator.queueValve(30000, true)
  //data.SHOULD_CUTDOWN = true;
  //actuator.cutDown();
}


#define minmin(a,b) ((a)<(b)?(a):(b))
#define maxmax(a,b) ((a)>(b)?(a):(b))

void Avionics::runHeaters() {
  float duty = 0;
  duty += maxmax((data.RB_HEAT_TEMP_THRESH - data.TEMP_INT) * data.RB_HEAT_TEMP_GAIN,0);
  duty += maxmax((data.RB_HEAT_CAP_NOMINAL - data.VOLTAGE_SUPERCAP_AVG)* data.RB_HEAT_CAP_GAIN, 0);
  duty += maxmax((((float)(millis()-data.RB_LAST)) - 2*data.RB_INTERVAL)/1000./3600.*data.RB_HEAT_COMM_GAIN, 0);

  duty = minmin(1, duty);
  duty = maxmax(0, duty);

  int dutyint = duty*data.RB_HEAT_MAX_DUTY;
  /*Serial.println((data.RB_HEAT_TEMP_THRESH - data.TEMP_INT));
  Serial.println((data.RB_HEAT_CAP_NOMINAL - data.VOLTAGE_SUPERCAP_AVG));
  Serial.println((((float)(millis()-data.RB_LAST)) - 2*data.RB_INTERVAL)/1000./3600.);*/
  //Serial.println(dutyint);
  analogWrite(36, dutyint);
  data.RB_HEAT_DUTY = dutyint;
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
// #ifdef HITL_ENABLED_FLAG
//   if(!simulateData()) alert("unable to simulate Data", true);
// #endif
  if(!processData())  alert("unable to process Data", true);

  /*if (data.TIME > 40000 && data.TIME < 40050) {
    actuator.queueBallast(30000, true);
  }*/
  //currentSensor.read_voltage(DIFF_12_13);
  //Serial.print("avg voltage: ");
  //uint32_t t0 = micros();
  //Serial.println(currentSensor.average_voltage_readings(DIFF_12_13, CURRENT_NUM_SAMPLES), 6);
  //uint32_t dt = micros() - t0;
  //Serial.println(dt);
  //delay(LOOP_INTERVAL);
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
  if(!runCharger()) alert("unable to run charger", true);
  if(!runValve())   alert("unable to run valve", true);
  if(!runBallast()) alert("unable to run ballast", true);
  if(!runCutdown()) alert("unable to run cutdown", true);
  if(!runLED())     alert("unable to run LED", true);
  runHeaters();
  //rumAndCoke();
  if(!runRadio()) alert("Unable to run payload", true);
}

/*
 * Function: logState
 * -------------------
 * This function logs the current data frame.
 */
void Avionics::logState() {
  uint32_t t0 = millis();
  //Serial.println("begin");
  //if(!log.log(&data, 1024)) alert("unable to log Data", true);
  //Serial.println("end");
  data.LOG_TIME = millis() - t0;
  if(!debugState())   alert("unable to debug state", true);
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
#ifndef RB_DISABLED_FLAG
  if(!data.VALVE_STATE && !data.POWER_STATE_RB && ((millis() - data.RB_LAST) > RB_RESTART_INTERVAL)) {
    Serial.println("RESTARTING RB WAT");
    RBModule.restart();
    data.POWER_STATE_RB = true;
  }
  Serial.println(millis()-data.RB_LAST);
  if(data.DEBUG_STATE && ((millis() - data.RB_LAST) < RB_DEBUG_INTERVAL)) return;
  if(!data.DEBUG_STATE && ((millis() - data.RB_LAST) < data.RB_INTERVAL)) return;
  Serial.println("not sending data at this time");
  if(compressData() < 0) alert("unable to compress Data", true);
  if(!sendSATCOMS())  {
    alert("unable to communicate over RB", true);
    // cooldown for 30 seconds
    uint32_t interval = data.RB_INTERVAL;
    if (data.DEBUG_STATE) interval = RB_DEBUG_INTERVAL;
    if (millis() > interval) {
      data.RB_LAST = millis() - interval + 30000;
    } else {
      data.RB_LAST = millis();
    }
  }
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
  Serial.println("calling initialize");
  bool ret = log.initialize();
  #ifdef WIPE_SD_CARD_YOLO
    ret = log.wipe();
  #endif
    Serial.println("done initialize");

  return ret;
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
    EEPROM.write(EEPROM_PAYLOAD, true);
    PCB.EEPROMWritelong(EEPROM_VALVE_ALT_LAST, data.VALVE_ALT_LAST);
    PCB.EEPROMWritelong(EEPROM_BALLAST_ALT_LAST, data.BALLAST_ALT_LAST);
#endif
#ifndef RESET_EEPROM_FLAG
  if(!EEPROM.read(EEPROM_ROCKBLOCK)) data.POWER_STATE_RB = false;
  if(!EEPROM.read(EEPROM_GPS)) data.POWER_STATE_GPS = false;
  if(!EEPROM.read(EEPROM_PAYLOAD)) data.POWER_STATE_PAYLOAD = false;
  double valveAltLast = PCB.EEPROMReadlong(EEPROM_VALVE_ALT_LAST);
  if (valveAltLast != 0) data.VALVE_ALT_LAST = valveAltLast;
  double ballastAltLast = PCB.EEPROMReadlong(EEPROM_BALLAST_ALT_LAST);
  if (ballastAltLast != 0) data.BALLAST_ALT_LAST = ballastAltLast;
#endif
  return true;
}

/*
 * Function: setup5VLine
 * -------------------
 * This function charges the superCap to 5 Volts.
 */
bool Avionics::setup5VLine() {
  while(sensors.getVoltageSuperCap() <= SUPER_CAP_VOLTAGE_NOMINAL) {
    delay(LOOP_INTERVAL);
    Serial.print("SuperCap is currently at: ");
    Serial.print(sensors.getVoltageSuperCap());
    Serial.println(" Volts.");
  }
  superCap.enable5VBoost();
  return true;
}

float Salt = 13000;
float Slift = 0.1;


/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
bool Avionics::readData() {
  data.LOOP_TIME                  = millis() - data.TIME;
  data.TIME                       = millis();
  data.VOLTAGE_PRIMARY            = sensors.getVoltagePrimary();
  data.VOLTAGE_SUPERCAP           = sensors.getVoltageSuperCap();
  data.CURRENT_TOTAL              = -2*currentSensor.average_voltage_readings(DIFF_10_11, CURRENT_NUM_SAMPLES);
  data.JOULES_TOTAL               += data.LOOP_TIME/1000.*data.VOLTAGE_PRIMARY*data.CURRENT_TOTAL/1000.;
  data.CURRENT_RB                 = -currentSensor.average_voltage_readings(DIFF_14_15, CURRENT_NUM_SAMPLES);
  data.MAX_CURRENT_CHARGING_LIMIT = superCap.getChargingLimit();
  float motcur = currentSensor.average_voltage_readings(DIFF_8_9, CURRENT_NUM_SAMPLES);
  data.CURRENT_MOTOR_VALVE        = (data.VALVE_STATE ? motcur : 0);
  data.CURRENT_MOTOR_BALLAST      = (data.BALLAST_STATE ? motcur : 0);
  data.CURRENT_MOTORS             = motcur;
  data.CURRENT_PAYLOAD            = -currentSensor.average_voltage_readings(DIFF_12_13, CURRENT_NUM_SAMPLES);
  data.TEMP_EXT                   = sensors.getDerivedTemp(EXT_TEMP_SENSOR);
  #ifdef SERIALSHITL
  shitlUpdate();
  #else
  data.RAW_TEMP_1                 = sensors.getRawTemp(1);
  data.RAW_TEMP_2                 = sensors.getRawTemp(2);
  data.RAW_TEMP_3                 = sensors.getRawTemp(3);
  data.RAW_TEMP_4                 = sensors.getRawTemp(4);
  data.RAW_PRESSURE_1             = sensors.getRawPressure(1);
  data.RAW_PRESSURE_2             = sensors.getRawPressure(2);
  data.RAW_PRESSURE_3             = sensors.getRawPressure(3);
  data.RAW_PRESSURE_4             = sensors.getRawPressure(4);
  #endif
  #ifdef JANKSHITL
  float v;
  if (Slift >= 0) {
    v = 5*sqrt(Slift);
  } else {
    v = -5*sqrt(-Slift);
  }
  Salt += v*data.LOOP_TIME/1000.;
  float p = 3.86651e-20 * pow(44330-Salt,5.255);
  data.RAW_PRESSURE_1             = p;
  data.RAW_PRESSURE_2             = p;
  data.RAW_PRESSURE_3             = p;
  data.RAW_PRESSURE_4             = p;
  #endif
  data.RAW_TEMP_1 = (isnan(data.RAW_TEMP_1) ? 0 : data.RAW_TEMP_1);
  data.RAW_TEMP_2 = (isnan(data.RAW_TEMP_2) ? 0 : data.RAW_TEMP_2);
  data.RAW_TEMP_3 = (isnan(data.RAW_TEMP_3) ? 0 : data.RAW_TEMP_3);
  data.RAW_TEMP_4 = (isnan(data.RAW_TEMP_4) ? 0 : data.RAW_TEMP_4);
  data.RAW_PRESSURE_1 = (isnan(data.RAW_PRESSURE_1) ? 0 : data.RAW_PRESSURE_1);
  data.RAW_PRESSURE_2 = (isnan(data.RAW_PRESSURE_2) ? 0 : data.RAW_PRESSURE_2);
  data.RAW_PRESSURE_3 = (isnan(data.RAW_PRESSURE_3) ? 0 : data.RAW_PRESSURE_3);
  data.RAW_PRESSURE_4 = (isnan(data.RAW_PRESSURE_4) ? 0 : data.RAW_PRESSURE_4);
  if (data.POWER_STATE_GPS && ((millis() - data.GPS_LAST) >= data.GPS_INTERVAL) && (!data.VALVE_STATE)) readGPS();
  return true;
}

/*
 * Function: readGPS
 * -------------------
 * This function reads data from the GPS module.
 */
bool Avionics::readGPS() {
  Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();
  Serial.println("READ GPS CALLED");
  gpsModule.smartDelay(GPS_LOCK_TIMEOUT);
  data.LAT_GPS          = gpsModule.getLatitude();
  data.LONG_GPS         = gpsModule.getLongitude();
  data.ALTITUDE_GPS     = gpsModule.getAltitude();
  data.HEADING_GPS      = gpsModule.getCourse();
  data.SPEED_GPS        = gpsModule.getSpeed();
  data.NUM_SATS_GPS     = gpsModule.getSats();
  data.GPS_LAST         = millis();
  Serial.println();Serial.println();Serial.println();Serial.println();
  return true;
}

#ifdef HITL_ENABLED_FLAG
bool Avionics::simulateData() {
  DataFrame simulation = HITL.readData();
  data.LOOP_TIME                 = millis() - data.TIME;
  data.TIME                      = millis();
  data.RAW_PRESSURE_1            = simulation.RAW_PRESSURE_1;
  data.RAW_PRESSURE_2            = simulation.RAW_PRESSURE_2;
  data.RAW_PRESSURE_3            = simulation.RAW_PRESSURE_3;
  data.RAW_PRESSURE_4            = simulation.RAW_PRESSURE_4;
  data.BMP_1_ENABLE              = simulation.BMP_1_ENABLE;
  data.BMP_2_ENABLE              = simulation.BMP_2_ENABLE;
  data.BMP_3_ENABLE              = simulation.BMP_3_ENABLE;
  data.BMP_4_ENABLE              = simulation.BMP_4_ENABLE;
  data.PRESS_BASELINE            = simulation.PRESS_BASELINE;
  data.BALLAST_ARM_ALT           = simulation.BALLAST_ARM_ALT;
  data.VALVE_SETPOINT            = simulation.VALVE_SETPOINT;
  data.VALVE_VENT_DURATION       = simulation.VALVE_VENT_DURATION;
  data.VALVE_VELOCITY_CONSTANT   = simulation.VALVE_VELOCITY_CONSTANT;
  data.BALLAST_SETPOINT          = simulation.BALLAST_SETPOINT;
  data.BALLAST_DROP_DURATION     = simulation.BALLAST_DROP_DURATION;
  data.BALLAST_VELOCITY_CONSTANT = simulation.BALLAST_VELOCITY_CONSTANT;
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

  data.VOLTAGE_SUPERCAP_AVG       = filter.getAvgVoltageSuperCap(data.VOLTAGE_SUPERCAP);
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
  float overpressure = analogRead(OP_PIN) * 1.2 / ((double)pow(2, 12)) * 3.2;
  float vref = analogRead(VR_PIN) * 1.2 / ((double)pow(2, 12)) * 3.2;
  float qty = (vref - 0.8)/2.;
  data.OVERPRESSURE = 498.1778/qty * (overpressure - (qty+0.5));

  data.OVERPRESSURE_VREF = vref;
  data.OVERPRESSURE_FILT          = op_filter.update(data.OVERPRESSURE);
  data.OVERPRESSURE_VREF_FILT          = op_vref_filter.update(data.OVERPRESSURE_VREF);
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
  noInterrupts();
    int numExecNow = numExecutions;
    numExecutions = 0;
  interrupts();
  bool success = true;
  computer.updateValveConstants(data.VALVE_SETPOINT, data.VALVE_VELOCITY_CONSTANT, data.VALVE_ALTITUDE_DIFF_CONSTANT, data.VALVE_LAST_ACTION_CONSTANT);
  computer.updateBallastConstants(data.BALLAST_SETPOINT, data.BALLAST_VELOCITY_CONSTANT, data.BALLAST_ALTITUDE_DIFF_CONSTANT, data.BALLAST_LAST_ACTION_CONSTANT);
  data.RE_ARM_CONSTANT   = computer.updateControllerConstants(data.BALLAST_ARM_ALT, data.INCENTIVE_THRESHOLD);
  data.VALVE_ALT_LAST    = computer.getAltitudeSinceLastVentCorrected(data.ALTITUDE_BAROMETER, data.VALVE_ALT_LAST);
  data.BALLAST_ALT_LAST  = computer.getAltitudeSinceLastDropCorrected(data.ALTITUDE_BAROMETER, data.BALLAST_ALT_LAST);
  data.VALVE_INCENTIVE   = computer.getValveIncentive(data.ASCENT_RATE, data.ALTITUDE_BAROMETER, data.VALVE_ALT_LAST);
  data.BALLAST_INCENTIVE = computer.getBallastIncentive(data.ASCENT_RATE, data.ALTITUDE_BAROMETER, data.BALLAST_ALT_LAST);
  if (!data.MANUAL_MODE && data.VALVE_INCENTIVE >= 1 && data.BALLAST_INCENTIVE >= 1) success = false;

  uint32_t INDEX = SPAG_CONTROLLER_INDEX;
  spagController.updateConstants(data.SPAG_CONSTANTS);
  SpaghettiController::Input spagInput;
  spagInput.h = data.ALTITUDE_BAROMETER;
  data.ACTIONS[INDEX] = 0;
  for (int k=0; k<numExecNow; k++) {
    spagController.update(spagInput);
    data.SPAG_STATE = spagController.getState();
    data.ACTIONS[INDEX] += spagController.getAction();
  }
  data.ACTION_TIME_TOTALS[2*INDEX] = data.ACTIONS[INDEX] < 0 ? data.ACTION_TIME_TOTALS[2*INDEX] - data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX];
  data.ACTION_TIME_TOTALS[2*INDEX+1] = data.ACTIONS[INDEX] > 0 ? data.ACTION_TIME_TOTALS[2*INDEX+1] + data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX+1];

  INDEX = SPAG2_CONTROLLER_INDEX;
  spag2Controller.updateConstants(data.SPAG2_CONSTANTS);
  SpaghettiController2::Input spag2Input;
  spag2Input.h = data.ALTITUDE_BAROMETER;
  data.ACTIONS[INDEX] = 0;
  for (int k=0; k<numExecNow; k++) {
    spag2Controller.update(spag2Input);
    data.SPAG2_STATE = spag2Controller.getState();
    data.ACTIONS[INDEX] += spag2Controller.getAction();
  }
  data.ACTION_TIME_TOTALS[2*INDEX] = data.ACTIONS[INDEX] < 0 ? data.ACTION_TIME_TOTALS[2*INDEX] - data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX];
  data.ACTION_TIME_TOTALS[2*INDEX+1] = data.ACTIONS[INDEX] > 0 ? data.ACTION_TIME_TOTALS[2*INDEX+1] + data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX+1];

  INDEX = LAS_CONTROLLER_INDEX;
  lasController.updateConstants(data.LAS_CONSTANTS);
  LasagnaController::Input lasInput;
  lasInput.h = data.ALTITUDE_BAROMETER;
  lasInput.op = isnan(data.OVERPRESSURE_FILT) ? 0 : data.OVERPRESSURE_FILT;
  data.ACTIONS[INDEX] = 0;
  for (int k=0; k<numExecNow; k++) {
    lasController.update(lasInput);
    data.LAS_STATE = lasController.getState();
    data.ACTIONS[INDEX] += lasController.getAction();
  }
  data.ACTION_TIME_TOTALS[2*INDEX] = data.ACTIONS[INDEX] < 0 ? data.ACTION_TIME_TOTALS[2*INDEX] - data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX];
  data.ACTION_TIME_TOTALS[2*INDEX+1] = data.ACTIONS[INDEX] > 0 ? data.ACTION_TIME_TOTALS[2*INDEX+1] + data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX+1];
  //Serial.print("numExecNow: ");
  //Serial.println(numExecNow);
  return success;
}


/*
 * Function: runCharger
 * -------------------
 * This function updates the ouput of the superCap charging circuit.
 */
bool Avionics::runCharger() {
  superCap.runChargerPID(data.RESISTOR_MODE, data.TEMP_INT);
  if(data.SYSTEM_POWER_STATE == 0) {
    if (data.VOLTAGE_SUPERCAP_AVG < 3.5) {
      data.POWER_STATE_LED = false;
      data.SYSTEM_POWER_STATE = 1;
    }
  }
  if(data.SYSTEM_POWER_STATE == 1) {
    if (data.VOLTAGE_SUPERCAP_AVG < 2.3) {
      data.POWER_STATE_RB = false;
      RBModule.shutdown();
      data.RB_LAST = millis();
      data.SYSTEM_POWER_STATE = 2;
    }
    if (data.VOLTAGE_SUPERCAP_AVG > 4.0) {
      data.POWER_STATE_LED = true;
      data.SYSTEM_POWER_STATE = 0;
    }
  }
  if(data.SYSTEM_POWER_STATE == 2) {
    if (data.VOLTAGE_SUPERCAP_AVG < 2.25) {
      superCap.disable5VBoost();
      data.SYSTEM_POWER_STATE = 3;
    }
    if (data.VOLTAGE_SUPERCAP_AVG > 3.5) {
      data.POWER_STATE_RB = true;
      RBModule.restart();
      data.RB_LAST = millis();
      data.SYSTEM_POWER_STATE = 1;
    }
  }
  if(data.SYSTEM_POWER_STATE == 3) {
    if (data.VOLTAGE_SUPERCAP_AVG > 3.0) {
      superCap.enable5VBoost();
      data.SYSTEM_POWER_STATE = 2;
    }
  }
  return true;
}

/*
 * Function: runValve
 * -------------------
 * This function actuates the valve based on the commanded action
 */
bool Avionics::runValve() {
  actuator.updateMechanicalConstants(data.VALVE_MOTOR_SPEED_OPEN, data.VALVE_MOTOR_SPEED_CLOSE, data.BALLAST_MOTOR_SPEED, data.VALVE_OPENING_DURATION, data.VALVE_CLOSING_DURATION);
  bool shouldAct = data.VALVE_INCENTIVE >= (1 + data.INCENTIVE_NOISE);
  uint32_t valveTime = data.VALVE_VENT_DURATION;
  if (data.CURRENT_CONTROLLER_INDEX != 0) {
    int numControllers = 3;
    if (data.CURRENT_CONTROLLER_INDEX <= numControllers && data.CURRENT_CONTROLLER_INDEX > 0) {
      shouldAct = data.ACTIONS[data.CURRENT_CONTROLLER_INDEX] < 0;
      if (shouldAct) valveTime = -data.ACTIONS[data.CURRENT_CONTROLLER_INDEX];
    }
  }
  if((shouldAct && actuator.getValveQueue() <= QUEUE_APPEND_THRESHOLD) || data.FORCE_VALVE) {
    data.VALVE_NUM_ATTEMPTS++;
    bool shouldValve = (!data.MANUAL_MODE || data.FORCE_VALVE);
    if(shouldValve) data.VALVE_NUM_ACTIONS++;
    if(!data.FORCE_VALVE) data.VALVE_ALT_LAST = data.ALTITUDE_BAROMETER;
    if(data.FORCE_VALVE) valveTime = data.VALVE_FORCE_DURATION;
    if(shouldValve) data.VALVE_TIME_TOTAL += valveTime;
    PCB.EEPROMWritelong(EEPROM_VALVE_ALT_LAST, data.VALVE_ALT_LAST);
    actuator.queueValve(valveTime, shouldValve);
    data.FORCE_VALVE = false;
  }
  data.VALVE_QUEUE = actuator.getValveQueue();
  data.VALVE_STATE = actuator.checkValve(data.CURRENT_MOTOR_VALVE);
  return true;
}

/*
 * Function: runBallast
 * -------------------
 * This function actuates the valve based on the commanded action
 */
bool Avionics::runBallast() {
  //Serial.println("runBallast() called");
  actuator.updateMechanicalConstants(data.VALVE_MOTOR_SPEED_OPEN, data.VALVE_MOTOR_SPEED_CLOSE, data.BALLAST_MOTOR_SPEED, data.VALVE_OPENING_DURATION, data.VALVE_CLOSING_DURATION);
  bool shouldAct = data.BALLAST_INCENTIVE >= (1 + data.INCENTIVE_NOISE);
  uint32_t ballastTime = data.BALLAST_DROP_DURATION;
  //Serial.print("data.CURRENT_CONTROLLER_INDEX: ");
  //Serial.println(data.CURRENT_CONTROLLER_INDEX);
  if (data.CURRENT_CONTROLLER_INDEX != 0) {
    //Serial.println("In first if body");
    int numControllers = sizeof(data.ACTIONS)/sizeof(data.ACTIONS[0]);
    if (data.CURRENT_CONTROLLER_INDEX <= 3 && data.CURRENT_CONTROLLER_INDEX > 0) {
      //Serial.println("In second if body");
      shouldAct = data.ACTIONS[data.CURRENT_CONTROLLER_INDEX] > 0;
      if (shouldAct) ballastTime = data.ACTIONS[data.CURRENT_CONTROLLER_INDEX];
    }
  }
  /*Serial.print("shouldAct is ");
  Serial.println(shouldAct);
  Serial.print("ballastTime is ");
  Serial.println(ballastTime);*/
  if((shouldAct && actuator.getBallastQueue() <= QUEUE_APPEND_THRESHOLD) || data.FORCE_BALLAST) {
    data.BALLAST_NUM_ATTEMPTS++;
    bool shouldBallast = (!data.MANUAL_MODE || data.FORCE_BALLAST);
    if(shouldBallast) data.BALLAST_NUM_ACTIONS++;
    if(!data.FORCE_BALLAST) data.BALLAST_ALT_LAST = data.ALTITUDE_BAROMETER;
    if(data.FORCE_BALLAST) ballastTime = data.BALLAST_FORCE_DURATION;
    if(shouldBallast) data.BALLAST_TIME_TOTAL += ballastTime;
    PCB.EEPROMWritelong(EEPROM_BALLAST_ALT_LAST, data.BALLAST_ALT_LAST);
    Serial.print("shouldBallast is ");
    Serial.println(shouldBallast);
    actuator.queueBallast(ballastTime, shouldBallast);
    data.FORCE_BALLAST = false;
  }
  data.BALLAST_QUEUE = actuator.getBallastQueue();
  data.BALLAST_NUM_OVERCURRENTS = actuator.getNumBallastOverCurrents();
  data.BALLAST_STATE = actuator.checkBallast(data.CURRENT_MOTOR_BALLAST, data.BALLAST_REVERSE_INTERVAL, data.BALLAST_STALL_CURRENT);
  return true;
}

/*
 * Function: runCutdown
 * -------------------
 * This function cuts down the payload if necessary.
 */
bool Avionics::runCutdown() {
  //Serial.println("Avionics.runCutdown called");
  if(data.SHOULD_CUTDOWN) {
    Serial.println("data.SHOULD_CUTDOWN was true");
    actuator.cutDown();
    data.SHOULD_CUTDOWN = false;
    data.CUTDOWN_STATE = true;
    alert("completed cutdown", false);
  }
  return true;
}


// 25.245315, -87.282412
// 18.628752, -71.779459
bool Avionics::checkInCuba() {
  if (data.LAT_GPS > 18.628752 && data.LAT_GPS < 24.152548 && data.LONG_GPS > -87.282412 && data.LONG_GPS < -71.779459 ) {
    return true;
  }
  return false;
}


void Avionics::rumAndCoke() {
  if (data.CUBA_NUMBER != 1973) return;
  bool now_in_cuba = checkInCuba();
  if (in_cuba && !now_in_cuba) {
    in_cuba = false;
  }
  if (!in_cuba && now_in_cuba) {
    in_cuba = true;
    cuba_timeout = millis() + 1000*3600;
  }
  /*if (in_cuba){
    Serial.println("CUBA WARNING");
    Serial.println(data.LOOP_TIME);
    Serial.println(cuba_timeout-millis());
  } else {
    Serial.println("NOT IN CUBA");
  }*/
  if (in_cuba && millis() > cuba_timeout) {
    data.SHOULD_CUTDOWN = true;
    runCutdown();
    //actuator.queueValve(1000000, true);
  }

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

uint32_t last_received = 0;

/*
 * Function: runPayload
 * -------------------
 * This function interfaces with the payload.
 */
bool Avionics::runRadio() {
  if (!data.POWER_STATE_RADIO) return true;
  radio.readyDataFrame();
  radio.addVariable(data.ALTITUDE_BAROMETER, -100, 25000, 16);
  radio.addVariable(data.ASCENT_RATE, -6, 6, 10);
  radio.addVariable(data.LAT_GPS,  -90, 90,  20);
  radio.addVariable(data.LONG_GPS, -180,  180, 21);
  radio.addVariable(data.VALVE_TIME_TOTAL / 1000, 0,  16383, 13);
  radio.addVariable(data.BALLAST_TIME_TOTAL / 1000, 0,  16383, 13);
  radio.addVariable(data.VOLTAGE_PRIMARY, 0,  6,  9);
  radio.addVariable(data.VOLTAGE_SUPERCAP,  0,  6,  9);
  radio.addVariable(data.TEMP_INT, -60, 60, 8);
  radio.addVariable(data.CURRENT_CONTROLLER_INDEX,  0,  3, 2);
  radio.addVariable(data.CURRENT_TOTAL, 0, 1500, 8);
  radio.addVariable(data.CURRENT_RB,  0,  2500, 7);
  radio.addVariable(data.CURRENT_MOTORS, 0,  500, 7);
  radio.addVariable(data.MANUAL_MODE, 0,  1,  1);
  radio.addVariable(data.OVERPRESSURE, -500,  500,  9);
  radio.addVariable(data.OVERPRESSURE_VREF, 0,  3.84,  7);
  radio.addVariable(data.LAS_STATE.v, -6, 6, 8);
  radio.addVariable(data.LAS_STATE.effort, -0.005, 0.005, 9);
  radio.addVariable(data.BALLAST_NUM_OVERCURRENTS, 0, 127, 6);
  bool got_sth = (millis()-last_received) < 15000;
  radio.addVariable(got_sth, 0, 1, 1);
  radio.setDataFrame();
  radio.run();
  if (radio.got_rb) {
    last_received = millis();
    for(uint16_t i = 0; i < COMMS_BUFFER_SIZE; i++) COMMS_BUFFER[i] = 0;
    memcpy(COMMS_BUFFER, radio.message, radio.parse_pos);
    if (COMMS_BUFFER[0] == 'l' &&
          COMMS_BUFFER[1] == 'm' &&
            COMMS_BUFFER[2] == 'f' &&
              COMMS_BUFFER[3] == 'a' &&
                COMMS_BUFFER[4] == 'o' &&
                  COMMS_BUFFER[5] == 'r' &&
                    COMMS_BUFFER[6] == 'i' &&
                      COMMS_BUFFER[7] == 'p') {
                        data.SHOULD_CUTDOWN = true;
                      }
    else {
      parseCommand(radio.parse_pos);
    }
    radio.got_rb = false;
  }
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
  Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();
  Serial.println("Waking up mr rockblock");
  Serial.println();Serial.println();
  RBModule.restart();
  int16_t ret = RBModule.writeRead(COMMS_BUFFER, data.COMMS_LENGTH);
  Serial.println("returned");
  Serial.println(ret);
  RBModule.shutdown();
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
  Serial.println("Got stuff");
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
    Serial.print(index);
    Serial.print(" -> ");
    if (index == CUTDOWN_INDEX && (strlen(CUTDOWN_COMMAND) == strlen(commandStrings[i])) && strncmp(commandStrings[i], CUTDOWN_COMMAND, strlen(commandStrings[i])) == 0) {
      data.SHOULD_CUTDOWN = true;
    }
    if (index < 0 || index > 80) return;
    char* charAfterNumbers;
    float commandValue = (float) strtod(commandStrings[i], &charAfterNumbers);
    if (*charAfterNumbers) return;
    Serial.println(commandValue);
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
  Serial.print("UPDATE COSNTANTS CALLED: ");
  Serial.print(index);
  Serial.print(" , ");
  Serial.println(value);
  if      (index ==  0) data.VALVE_ALT_LAST = value; // Altitude During Last Venting Event | meters
  else if (index ==  1) data.BALLAST_ALT_LAST = value; // Altitude During Last Ballast Event | meters
  else if (index ==  2) data.VALVE_SETPOINT = value; // Valve Setpoint | meters
  else if (index ==  3) data.BALLAST_SETPOINT = value; // Ballast Setpoint | meters
  else if (index ==  4) data.BALLAST_ARM_ALT = value; // Ballast Arming Altitude | meters
  else if (index ==  5) data.INCENTIVE_THRESHOLD = value; // Incentive threshold
  else if (index ==  6) data.VALVE_VELOCITY_CONSTANT = value; // Valve Speed Constant
  else if (index ==  7) data.VALVE_ALTITUDE_DIFF_CONSTANT = 1.0 / value; // Valve Altitude Difference Constant | 1 / X
  else if (index ==  8) data.VALVE_LAST_ACTION_CONSTANT = 1.0 / value; // Valve Last Action Constant | 1 / X
  else if (index ==  9) data.BALLAST_VELOCITY_CONSTANT = value; // Ballast Speed Constant
  else if (index == 10) data.BALLAST_ALTITUDE_DIFF_CONSTANT = 1.0 / value; // Ballast Altitude Difference Constant | 1 / X
  else if (index == 11) data.BALLAST_LAST_ACTION_CONSTANT = 1.0 / value; // Ballast last action constant | 1 / X
  else if (index == 12) data.VALVE_VENT_DURATION = value * 1000; // Valve duration | seconds
  else if (index == 13) data.BALLAST_DROP_DURATION = value * 1000; // Ballast duration | seconds
  else if (index == 14) data.PRESS_BASELINE = value; // Pressure baseline | Pascals
  else if (index == 15) data.BALLAST_REVERSE_INTERVAL = value * 1000; // Ballast reverse timeout | seconds
  else if (index == 16) data.BALLAST_STALL_CURRENT = value; // Ballast stall current | milliamps
  else if (index == 17) data.VALVE_MOTOR_SPEED_OPEN = value; // Valve motor speed open | Between 0 and 255
  else if (index == 18) data.VALVE_MOTOR_SPEED_CLOSE = value; // Valve motor speed close | Between 0 and 255
  else if (index == 19) data.BALLAST_MOTOR_SPEED = value; // Ballast motor speed | Between 0 and 255
  else if (index == 20) data.VALVE_OPENING_DURATION = value * 1000; // Valve opening timeout | seconds
  else if (index == 21) data.VALVE_CLOSING_DURATION = value * 1000; // Valve closing timeout | seconds
  else if (index == 22) data.POWER_STATE_LED = value; // LED Power State | bool (0 or 1)
  else if (index == 23) data.RB_INTERVAL = value * 1000; // RB communication interval | seconds
  else if (index == 24) data.GPS_INTERVAL = value * 1000; // GPS communication interval | seconds
  else if (index == 25) parseManualCommand(value); // Manual mode | 0: automatic; 1: manual
  else if (index == 26) parseReportCommand(value); // Report mode | 0: standard; 1: extended; 2: readback
  else if (index == 27) parseSensorsCommand(value); // Active sensors | u8; little endian
  else if (index == 28) parseValveCommand(value * 1000); // Vent now for X seconds (0 to clear queue) | seconds
  else if (index == 29) parseBallastCommand(value * 1000); // Ballast now for X seconds (0 to clear queue) | seconds
  else if (index == 30) parseRockBLOCKPowerCommand(value); // RB Power State | 0: shutdown; 1: restart
  else if (index == 31) parseGPSPowerCommand(value); // GPS Power State | 0: shutdown; 1: restart; 2: hot start
  else if (index == 32) parsePayloadPowerCommand(value); // Payload Power State | 0: shutdown; 1: restart
  else if (index == 33) data.CURRENT_CONTROLLER_INDEX = value; // Controller Index | 0: Legacy; 1: Spaghetti; 2: Spaghetti2; 3: Lasagna
  else if (index == 34) data.SPAG_CONSTANTS.freq      = value; // Spaghetti Frequency
  else if (index == 35) data.SPAG_CONSTANTS.k         = value; // Spaghetti K Value
  else if (index == 36) data.SPAG_CONSTANTS.b_dldt    = value; // Spaghetti B DLDT
  else if (index == 37) data.SPAG_CONSTANTS.v_dldt    = value; // Spaghetti V DLDT
  else if (index == 38) data.SPAG_CONSTANTS.rate_min  = value; // Spaghetti Rate Min
  else if (index == 39) data.SPAG_CONSTANTS.rate_max  = value; // Spaghetti Rate Max
  else if (index == 40) data.SPAG_CONSTANTS.b_tmin    = value; // Spaghetti B TMIN
  else if (index == 41) data.SPAG_CONSTANTS.v_tmin    = value; // Spaghetti V TMIN
  else if (index == 42) data.SPAG_CONSTANTS.h_cmd     = value; // Spaghetti H CMD
  else if (index == 43) data.SPAG2_CONSTANTS.freq               = value; // Spaghetti2 FREQ
  else if (index == 44) data.SPAG2_CONSTANTS.k                  = value; // Spaghetti2 K
  else if (index == 45) data.SPAG2_CONSTANTS.b_dldt             = value; // Spaghetti2 B DLDT
  else if (index == 46) data.SPAG2_CONSTANTS.v_dldt             = value; // Spaghetti2 V DLDT
  else if (index == 47) data.SPAG2_CONSTANTS.rate_min           = value; // Spaghetti2 RATE MIN
  else if (index == 48) data.SPAG2_CONSTANTS.rate_max           = value; // Spaghetti2 RATE MAX
  else if (index == 49) data.SPAG2_CONSTANTS.b_tmin             = value; // Spaghetti2 B TMIN
  else if (index == 50) data.SPAG2_CONSTANTS.v_tmin             = value; // Spaghetti2 V TMIN
  else if (index == 51) data.SPAG2_CONSTANTS.h_cmd              = value; // Spaghetti2 H CMD
  else if (index == 52) data.SPAG2_CONSTANTS.v_ss_error_thresh  = value; // Spaghetti2 V SS ERROR THRESH
  else if (index == 53) data.SPAG2_CONSTANTS.b_ss_error_thresh  = value; // Spaghetti2 B SS ERROR THRESH
  else if (index == 54) data.SPAG2_CONSTANTS.ascent_rate_thresh = value; // Spaghetti2 ASCENT RATE THRESH
  else if (index == 55) data.SPAG2_CONSTANTS.kfuse              = value; // Spaghetti2 KFUSE
  else if (index == 56) data.SPAG2_CONSTANTS.kfuse_v            = value; // Spaghetti2 KFUSE V
  else if (index == 57) data.LAS_CONSTANTS.freq            = value; // Lasagna FREQ
  else if (index == 58) data.LAS_CONSTANTS.k_v             = value; // Lasagna K V
  else if (index == 59) data.LAS_CONSTANTS.k_h             = value; // Lasagna K H
  else if (index == 60) data.LAS_CONSTANTS.b_dldt          = value; // Lasagna B dLdT
  else if (index == 61) data.LAS_CONSTANTS.v_dldt_a        = value; // Lasagna V dLdT A
  else if (index == 62) data.LAS_CONSTANTS.v_dldt_b        = value; // Lasagna V dLdT B
  else if (index == 63) data.LAS_CONSTANTS.b_tmin          = value; // Lasagna B tMin
  else if (index == 64) data.LAS_CONSTANTS.v_tmin          = value; // Lasagna V tMin
  else if (index == 65) data.LAS_CONSTANTS.h_cmd           = value; // Lasagna H Cmd
  else if (index == 66) data.LAS_CONSTANTS.kfuse           = value; // Lasagna kFuse
  else if (index == 67) data.LAS_CONSTANTS.kfuse_val       = value; // Lasagna kFuse V
  else if (index == 68) data.LAS_CONSTANTS.ss_error_thresh = value; // Lasagna SS Error Thresh
  else if (index == 69) data.RB_HEAT_TEMP_THRESH           = value; // RB Heat Temp Thresh
  else if (index == 70) data.RB_HEAT_TEMP_GAIN             = value; // RB Heat Temp Gain
  else if (index == 71) data.RB_HEAT_COMM_GAIN             = value; // RB Heat Comm Gain
  else if (index == 72) data.RB_HEAT_CAP_GAIN              = value; // RB Heat Cap Gain
  else if (index == 73) data.RB_HEAT_MAX_DUTY              = value; // RB Heat Max Duty
  else if (index == 74) data.RB_HEAT_CAP_NOMINAL           = value; // RB Heat Cap Nominal | V
  else if (index == 75) { // Cuba Number
    data.CUBA_NUMBER           = (int)value;
    in_cuba = false;
    cuba_timeout = millis() + 3600*1000;
  }
  else if (index == 76) data.RESISTOR_MODE           = (int)value; // Resistor mode
  else if (index == 77) parseRadioPowerCommand(value);
  else if (index == 79) { // GPS mode
    int GPS_MODE = (int) value;
    if (GPS_MODE == 0 || GPS_MODE == 1) {
      gpsModule.GPS_MODE = GPS_MODE;
      gpsModule.restart();
    }
  }
}

/*
 * Function: parseManualCommand
 * -------------------
 * This function parses the manual mode command.
 */
void Avionics::parseManualCommand(bool command) {
  actuator.clearValveQueue();
  actuator.clearBallastQueue();
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
  if(command == 0) actuator.clearValveQueue();
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
  if(command == 0) actuator.clearBallastQueue();
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
    RBModule.restart();
  }
  else if (!command) {
    data.POWER_STATE_RB = false;
    RBModule.shutdown();
  }
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
 * Function: parseResistorPowerCommand
 * -------------------
 * This function parses the Resistor power command.
 */
void Avionics::parseResistorPowerCommand(uint8_t command) {
  data.RESISTOR_MODE = command;
}

/*
 * Function: parsePayloadPowerCommand
 * -------------------
 * This function parses the Payload power command.
 */
void Avionics::parsePayloadPowerCommand(bool command) {
  if (command && !data.POWER_STATE_PAYLOAD) {
    data.POWER_STATE_PAYLOAD = true;
    digitalWrite(57, HIGH);
  }
  else if (!command) {
    data.POWER_STATE_PAYLOAD = false;
    digitalWrite(57, LOW);
  }
}

/* Wow I'm really not a fan of code decomposition - Joan. */
void Avionics::parseRadioPowerCommand(bool command) {
  if (command && !data.POWER_STATE_RADIO) {
    data.POWER_STATE_RADIO = true;
    radio.restart();
  }
  else if (!command) {
    data.POWER_STATE_RADIO = false;
    radio.shutdown();
  }
}

/*
 * Function: debugState
 * -------------------
 * This function provides debuging information.
 */
bool Avionics::debugState() {
  printState();
  //if(data.DEBUG_STATE) printState();
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
  actuator.clearBallastOverCurrents();
  data.VALVE_NUM_ACTIONS = 0;
  data.BALLAST_NUM_ACTIONS = 0;
  data.VALVE_NUM_ATTEMPTS = 0;
  data.BALLAST_NUM_ATTEMPTS = 0;
  data.LOOP_TIME_MAX = 0;
  int len = sizeof(data.ACTION_TIME_TOTALS)/sizeof(data.ACTION_TIME_TOTALS[0]);
  for(int i=0; i < len; i++){
    data.ACTION_TIME_TOTALS[i] = 0;
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
  lengthBits += compressVariable(data.CURRENT_CONTROLLER_INDEX,             0,    3,        2,  lengthBits);
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
  lengthBits += compressVariable(data.MAX_CURRENT_CHARGING_LIMIT,            0,    3,       2,  lengthBits);
  lengthBits += compressVariable(data.SYSTEM_POWER_STATE,                    0,    3,       2,  lengthBits);
  lengthBits += compressVariable(data.TEMP_INT,                             -85,   65,      9,  lengthBits);
  lengthBits += compressVariable(data.JOULES_TOTAL,                          0,    1572863, 18, lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_PRIMARY,                       0,    6,       9,  lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_SUPERCAP_AVG,                  0,    6,       9,  lengthBits);
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
  lengthBits += compressVariable(data.TEMP_EXT,                             -100,  30,      8,  lengthBits);
  lengthBits += compressVariable(data.LOOP_TIME_MAX,                         0,    10239,   10, lengthBits);
  lengthBits += compressVariable(data.RB_SENT_COMMS,                         0,    8191,    13, lengthBits);
  lengthBits += compressVariable(data.RESISTOR_MODE,                         0,    4,       3,  lengthBits);
  lengthBits += compressVariable(data.MANUAL_MODE,                           0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.REPORT_MODE,                           0,    2,       2,  lengthBits);
  lengthBits += compressVariable(data.SHOULD_REPORT,                         0,    1,       1,  lengthBits);
  if (data.SHOULD_REPORT || data.REPORT_MODE != 0) {
    lengthBits += compressVariable(data.POWER_STATE_LED,                     0,    1,       1,  lengthBits); // LED Power state
    lengthBits += compressVariable(data.POWER_STATE_RB,                      0,    1,       1,  lengthBits); // RB Power State
    lengthBits += compressVariable(data.POWER_STATE_GPS,                     0,    1,       1,  lengthBits); // GPS Power State
    lengthBits += compressVariable(data.POWER_STATE_PAYLOAD,                 0,    1,       1,  lengthBits); // Payload Power State
    lengthBits += compressVariable(data.NUM_SATS_GPS,                        0,    15,      3,  lengthBits);
    lengthBits += compressVariable(data.INCENTIVE_NOISE,                     0,    4,       8,  lengthBits);
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
    lengthBits += compressVariable(data.ACTION / 1000,              -1023, 1023,     7,  lengthBits);
    lengthBits += compressVariable(data.VALVE_INCENTIVE,            -50,   10,       12, lengthBits);
    lengthBits += compressVariable(data.BALLAST_INCENTIVE,          -50,   10,       12, lengthBits);
    lengthBits += compressVariable(data.RE_ARM_CONSTANT,             0,    4,        8,  lengthBits);
    lengthBits += compressVariable(data.VALVE_ALT_LAST,             -2000, 50000,    11, lengthBits);
    lengthBits += compressVariable(data.BALLAST_ALT_LAST,           -2000, 50000,    11, lengthBits);
    lengthBits += compressVariable(data.SPAG_STATE.effort*1000,            -2, 2, 12, lengthBits);
    lengthBits += compressVariable(data.SPAG2_STATE.effort*1000,           -2, 2, 12, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.v,           -10,     10,   11, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.fused_v,     -10,     10,   11, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.effort,                  -2,       2,  11, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.v_cmd,                 -10,      10,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[2]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[3]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[4]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[5]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[6]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[7]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.RB_HEAT_DUTY,        0,     255,   8, lengthBits);
    lengthBits += compressVariable(data.RB_HEAT_TEMP_THRESH,        -100,     100,   8, lengthBits);
    lengthBits += compressVariable(in_cuba,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(cuba_timeout,                        0,    4000000,       10,  lengthBits);
    lengthBits += compressVariable(data.OVERPRESSURE_FILT,    -500,     500,   9, lengthBits);
    lengthBits += compressVariable(data.OVERPRESSURE_VREF_FILT,    0,     3.84,   8, lengthBits);
  }
  if (data.SHOULD_REPORT || data.REPORT_MODE == 2) {
    lengthBits += compressVariable(data.RB_INTERVAL / 1000,                  0,    1023,    10, lengthBits); // RB communication interval
    lengthBits += compressVariable(data.GPS_INTERVAL / 1000,                 0,    1023,    10, lengthBits); // GPS communication interval
    lengthBits += compressVariable(data.PRESS_BASELINE,                      0,    131071,  17, lengthBits); // Pressure baseline
    lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,                 0,    4,       3,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_ARM_ALT,                    -2000, 40000,   16, lengthBits); // Ballast Arming Altitude
    lengthBits += compressVariable(data.BALLAST_REVERSE_INTERVAL / 1000,     0,    1599,    4,  lengthBits); // Ballast reverse interval
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
    // spaghetti readback
    lengthBits += compressVariable(data.SPAG_CONSTANTS.k,                      0,      2,    6,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.b_dldt*1000,            0,      100,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.v_dldt*1000,            0,      100,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.rate_min*1000,          0,      0.2,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.rate_max*1000,          0,      2,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.b_tmin,                 0,      31,    5,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.v_tmin,                 0,      31,    5,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.h_cmd,                  -2000,    40000,    11,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.k,                     0,      2,    6,      lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.b_dldt*1000,           0,      100,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.v_dldt*1000,           0,      100,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.rate_min*1000,         0,      0.2,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.rate_max*1000,         0,      2,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.b_tmin,                0,      31,    5,   lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.v_tmin,                0,      31,    5,   lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.h_cmd,                 -2000,    40000,    11,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.v_ss_error_thresh,     0,      3000,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.b_ss_error_thresh,     0,      3000,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.ascent_rate_thresh,    0,      10,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.kfuse,                 0,      30,    6,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.kfuse_v,               0,      1,    4,  lengthBits);

    lengthBits += compressVariable(data.LAS_CONSTANTS.k_v,                   0,      .01,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.k_h,                   0,      .01,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.b_dldt*1000,           0,      100,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_dldt_a*1000,           0,      100,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_dldt_b*1000,           0,      100,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.b_tmin,                0,       20,   4,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_tmin,                0,       20,   4,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.h_cmd,                 0,    20000,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.kfuse,                 0,        30,  6,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.kfuse_val,             0,        1,   4,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.ss_error_thresh,       0,     3000,   8,  lengthBits);
  }
  lengthBits += 8 - (lengthBits % 8);
  lengthBytes = lengthBits / 8;
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
  //if (data.LOOP_NUMBER % 500 != 0) return;
  // Serial.print("CURRENT MOTORS: ");
  //   Serial.print(data.CURRENT_MOTORS);
  //   Serial.print(" ");
  //     Serial.print(data.CURRENT_TOTAL);
  //     Serial.print(" ");
  //       Serial.print(data.CURRENT_RB);
  //     Serial.print(" ");
  //       Serial.print(data.CURRENT_PAYLOAD);
  //     Serial.print(" ");
  //     //Serial.print(data.CURRENT_MOTOR_BALLAST_AVG);
  //     Serial.println();
      //return;
  Serial.print("!!!!!!manual mode ");
  Serial.println(data.MANUAL_MODE);
  //return;
  Serial.print("Altitude: ");
  Serial.println(data.ALTITUDE_BAROMETER);
  Serial.print("Primary voltage: ");
  Serial.println(data.VOLTAGE_PRIMARY);
  // Serial.print("System current: ");
  // Serial.println(data.CURRENT_TOTAL);
  Serial.print("System current: ");
  Serial.println(data.CURRENT_TOTAL);
  Serial.print("Payload current: ");
  Serial.println(data.CURRENT_PAYLOAD);
  Serial.print("Motor current: ");
  Serial.println(data.CURRENT_MOTORS);
  Serial.print("RB current: ");
  Serial.println(data.CURRENT_RB);
  Serial.print("Overpressure: ");
  Serial.println(data.OVERPRESSURE_FILT);
  Serial.print("Overpressure vref: ");
  Serial.println(data.OVERPRESSURE_VREF_FILT);
  //return;
  // Serial.print("MANUAL_MODE: ");
  // Serial.println(data.MANUAL_MODE);

  // Serial.print(" RAW_TEMP_1:");
  // Serial.print(data.RAW_TEMP_1);
  // Serial.print(',');
  // Serial.print(" RAW_TEMP_2:");
  // Serial.print(data.RAW_TEMP_2);
  // Serial.print(',');
  // Serial.print(" RAW_TEMP_3:");
  // Serial.print(data.RAW_TEMP_3);
  // Serial.print(',');
  // Serial.print(" RAW_TEMP_4:");
  // Serial.print(data.RAW_TEMP_4);
  // Serial.print(',');
  /*Serial.print("RAW_PRESSURE_1: ");
  Serial.println(data.RAW_PRESSURE_1);
  //Serial.print(',');
  Serial.print("RAW_PRESSURE_2: ");
  Serial.println(data.RAW_PRESSURE_2);
  //Serial.print(',');
  Serial.print("RAW_PRESSURE_3: ");
  Serial.println(data.RAW_PRESSURE_3);
  //Serial.print(',');
  Serial.print("RAW_PRESSURE_4: ");
  Serial.println(data.RAW_PRESSURE_4);*/
  Serial.println();
  //return;

  Serial.print("MANUAL_MODE: ");
  Serial.println(data.MANUAL_MODE);
  Serial.print("CONTROLLER: ");
  Serial.println(data.CURRENT_CONTROLLER_INDEX);
  Serial.print("SPAG_EFFORT: ");
  Serial.println(data.SPAG_STATE.effort*1000);
  Serial.print("SPAG2_EFFORT: ");
  Serial.println(data.SPAG2_STATE.effort*1000);
  Serial.print("LAS_EFFORT: ");
  Serial.println(data.LAS_STATE.effort*1000);
  Serial.print("VALVE_INCENTIVE_LEGACY: ");
  Serial.println(data.VALVE_INCENTIVE);
  Serial.print("BALLAST_INCENTIVE_LEGACY: ");
  Serial.println(data.BALLAST_INCENTIVE);
  Serial.print("VALVE QUEUE: ");
  Serial.println(data.VALVE_QUEUE);
  Serial.print("BALLAST QUEUE: ");
  Serial.println(data.BALLAST_QUEUE);
  Serial.print("ACTION: ");
  Serial.println(data.ACTION);
  Serial.print("HEATER THRESHOLD: ");
  Serial.println(data.RB_HEAT_TEMP_THRESH);

  Serial.println("action crap");
  for (int kk = 0; kk < 4; kk++ ) {
    Serial.print(data.ACTION_TIME_TOTALS[2*kk]);
    Serial.print(" ");
      Serial.print(data.ACTION_TIME_TOTALS[2*kk+1]);
      Serial.println();
  }
      Serial.println();
  Serial.println();
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
  Serial.print(" ACTION:");
  Serial.print(data.ACTION);
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
  Serial.print(" MAX_CURRENT_CHARGING_LIMIT:");
  Serial.print(data.MAX_CURRENT_CHARGING_LIMIT);
  Serial.print(',');
  Serial.print(" SYSTEM_POWER_STATE:");
  Serial.print(data.SYSTEM_POWER_STATE);
  Serial.print(',');
  Serial.print(" TEMP_INT:");
  Serial.print(data.TEMP_INT);
  Serial.print(',');
  Serial.print(" JOULES_TOTAL:");
  Serial.print(data.JOULES_TOTAL);
  Serial.print(',');
  Serial.print(" VOLTAGE_PRIMARY:");
  Serial.print(data.VOLTAGE_PRIMARY);
  Serial.print( ',');
  Serial.print(" VOLTAGE_SUPERCAP_AVG:");
  Serial.print(data.VOLTAGE_SUPERCAP_AVG);
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
  return;
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
  Serial.print(" TEMP_EXT:");
  Serial.print(data.TEMP_EXT);
  Serial.print(',');
  Serial.print(" LOOP_TIME_MAX:");
  Serial.print(data.LOOP_TIME_MAX);
  Serial.print(',');
  Serial.print(" RB_SENT_COMMS:");
  Serial.print(data.RB_SENT_COMMS);
  Serial.print(',');
  Serial.print(" RESISTOR_MODE:");
  Serial.print(data.RESISTOR_MODE);
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
  Serial.print(" POWER_STATE_PAYLOAD:");
  Serial.print(data.POWER_STATE_PAYLOAD);
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
  Serial.print(" VALVE_ALT_LAST:");
  Serial.print(data.VALVE_ALT_LAST);
  Serial.print(',');
  Serial.print(" BALLAST_ALT_LAST:");
  Serial.print(data.BALLAST_ALT_LAST);
  Serial.print(',');
  // START OF CONTROLLER SWITCHING ADDITION
  Serial.print(" CURRENT_CONTROLLER_INDEX:");
  Serial.print(data.CURRENT_CONTROLLER_INDEX);
  Serial.print(',');
  Serial.print(',');
 // END OF CONTROLLER SWITCHING ADDITION
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
  Serial.print(" RB_INTERVAL:");
  Serial.print(data.RB_INTERVAL);
  Serial.print(',');
  Serial.print(" GPS_INTERVAL:");
  Serial.print(data.GPS_INTERVAL);
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
  Serial.print(" VOLTAGE_SUPERCAP:");
  Serial.print(data.VOLTAGE_SUPERCAP);
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
  Serial.print(",");
  Serial.print("\n\r");
  Serial.print("\n\r");
}
