#include "Avionics.h"

/*
 * Function: runCharger
 * -------------------
 * This function updates the ouput of the superCap charging circuit.
 */
bool Avionics::runCharger() {
  superCap.runChargerPID(data.RESISTOR_MODE, data.TEMP_INT);
  if(data.SYSTEM_POWER_STATE == 0) {
    if (data.VOLTAGE_SUPERCAP_AVG < 4.25) {
      data.POWER_STATE_LED = false;
      data.SYSTEM_POWER_STATE = 1;
    }
  }
  if(data.SYSTEM_POWER_STATE == 1) {
    if (data.VOLTAGE_SUPERCAP_AVG < 3.0) {
      data.POWER_STATE_RB = false;
      Serial.println("shutting down RB, possibly mid comm, sad");
      RBModule.shutdown();
      data.RB_LAST = millis();
      data.SYSTEM_POWER_STATE = 2;
    }
    if (data.VOLTAGE_SUPERCAP_AVG > 4.5) {
      data.POWER_STATE_LED = true;
      data.SYSTEM_POWER_STATE = 0;
    }
  }
  if(data.SYSTEM_POWER_STATE == 2) {
    if (data.VOLTAGE_SUPERCAP_AVG < 2.5) {
      actuator.pause();
      superCap.disable5VBoost();
      data.SYSTEM_POWER_STATE = 3;
    }
    if (data.VOLTAGE_SUPERCAP_AVG > 4.5) {
      data.POWER_STATE_RB = true;
      Serial.println("rb good to go again");
      data.RB_LAST = millis();
      data.SYSTEM_POWER_STATE = 1;
    }
  }
  if(data.SYSTEM_POWER_STATE == 3) {
    if (data.VOLTAGE_SUPERCAP_AVG > 4) {
      Serial.println("enabling 5 V boost");
      actuator.play();
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
    int numControllers = 3;
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
    if(!data.FORCE_BALLAST && data.CURRENT_CONTROLLER_INDEX == 0) data.BALLAST_ALT_LAST = data.ALTITUDE_BAROMETER;
    if(data.FORCE_BALLAST) ballastTime = data.BALLAST_FORCE_DURATION;
    if(shouldBallast) data.BALLAST_TIME_TOTAL += ballastTime;
    PCB.EEPROMWritelong(EEPROM_BALLAST_ALT_LAST, data.BALLAST_ALT_LAST);
    Serial.print("shouldBallast is ");
    Serial.println(shouldBallast);
    actuator.queueBallast(ballastTime, shouldBallast);
    data.FORCE_BALLAST = false;
  }
  data.BALLAST_DIRECTION = actuator.getBallastDirection();
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
  if ((millis() - data.TIME_LAST_CUTDOWN) < 60000) {
    return true;
  }
  if(data.SHOULD_CUTDOWN) {
    Serial.println("data.SHOULD_CUTDOWN was true");
    actuator.cutDown();
    data.SHOULD_CUTDOWN = false;
    data.CUTDOWN_STATE = true;
    data.TIME_LAST_CUTDOWN = millis();
    alert("completed cutdown", false);
  }
  return true;
}


// 25.245315, -87.282412
// 18.628752, -71.779459
bool Avionics::checkInCuba() {
  if (data.LAT_GPS > data.BB_LAT1 && data.LAT_GPS < data.BB_LAT2 && data.LONG_GPS > data.BB_LON1 && data.LONG_GPS < data.BB_LON2 ) {
    return true;
  }
  return false;
}

void Avionics::timedCutdown() {
  if (data.TIMED_CUTDOWN_ENABLE) {
    Serial.print("Timed cutdown is enabled... will cutdown in ");
    Serial.println(data.TIMED_CUTDOWN_MILLIS-millis());
  }
  if (data.TIMED_CUTDOWN_ENABLE && millis() > data.TIMED_CUTDOWN_MILLIS) {
    data.SHOULD_CUTDOWN = true;
    data.TIMED_CUTDOWN_ENABLE = false;
    runCutdown();
  }
}


void Avionics::rumAndCoke() {
  if (!data.GEOFENCED_CUTDOWN_ENABLE) return;
  bool now_in_cuba = checkInCuba();
  if (data.IN_CUBA && !now_in_cuba) {
    data.IN_CUBA = false;
  }
  if (!data.IN_CUBA && now_in_cuba) {
    data.IN_CUBA = true;
    data.CUBA_TIMEOUT = millis() + 1000*data.CUBA_MAX_SECONDS;
  }
  //Serial.print("Geofenced cutdown is enabled... ");
  if (data.IN_CUBA) { Serial.println("I think I'm in Cuba"); }
  else { /*Serial.println("but I live in a free country");*/ }
  /*if (in_cuba){
    Serial.println("CUBA WARNING");
    Serial.println(data.LOOP_TIME);
    Serial.println(cuba_timeout-millis());
  } else {
    Serial.println("NOT IN CUBA");
  }*/
  if (data.IN_CUBA && millis() > data.CUBA_TIMEOUT) {
    data.SHOULD_CUTDOWN = true;
    runCutdown();
    //actuator.queueValve(1000000, true);
  }

}

void Avionics::runDeadMansSwitch() {
  if (!data.DEADMAN_ENABLED) return;

  /* Note: it will be repeatedly called, but runCutdown() runs at most once per minute. */
  if ((millis() - data.TIME_LAST_COMM) > data.DEADMAN_TIME) {
    data.SHOULD_CUTDOWN = true;
    runCutdown();
  }
}

/*
 * Function: runLED
 * -------------------
 * This function blinks the 1HZ LED required by the FAA.
 */
bool Avionics::runLED() {
	pinMode(25, OUTPUT);
	digitalWrite(25, HIGH);
  /*if (data.POWER_STATE_LED && (uint32_t(millis() / 1000.0) % 2 == 1)) PCB.runLED(true);
  else PCB.runLED(false);*/
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
