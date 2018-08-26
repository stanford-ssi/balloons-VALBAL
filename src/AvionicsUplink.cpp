#include "Avionics.h"

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
    if (index < 0 || index > 128) return;
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
  else if (index == 69) data.LAS_CONSTANTS.v_limit         = value; // Lasanga V Limit
  else if (index == 70) data.LAS_CONSTANTS.equil_h_thresh  = value; // Lasagna Equilibrium H Thresh
  else if (index == 71) data.LAS_CONSTANTS.launch_h_thresh = value; // Lasagna Launch H Thresh
  else if (index == 72) data.HEATER_CONSTANTS.temp_thresh           = value; // RB Heat Temp Thresh
  else if (index == 73) data.HEATER_CONSTANTS.temp_gain             = value; // RB Heat Temp Gain
  else if (index == 74) data.HEATER_CONSTANTS.comm_gain             = value; // RB Heat Comm Gain
  else if (index == 75) data.HEATER_CONSTANTS.cap_gain              = value; // RB Heat Cap Gain
  else if (index == 76) data.HEATER_CONSTANTS.max_duty              = value; // RB Heat Max Duty
  else if (index == 77) data.HEATER_CONSTANTS.cap_nominal           = value; // RB Heat Cap Nominal | V
  else if (index == 78) { // Cuba Number
    data.GEOFENCED_CUTDOWN_ENABLE           = (bool)value;
    data.IN_CUBA = false;
    data.CUBA_TIMEOUT = millis() + 1000*data.CUBA_MAX_SECONDS;
  }
  else if (index == 79) data.RESISTOR_MODE           = (int)value; // Resistor mode
  else if (index == 80) parseRadioPowerCommand(value);
  else if (index == 81) { // GPS mode
    int GPS_MODE = (int) value;
    if (GPS_MODE == 0 || GPS_MODE == 1) {
      gpsModule.GPS_MODE = GPS_MODE;
      gpsModule.restart();
    }
  }
  else if (index == 82) data.MAX_CONSENSUS_DEVIATION = value;
  else if (index == 83) data.BMP_REJECTION_ENABLED = (bool)value;
  else if (index == 84) data.MAX_TIME_WITHOUT_SENSORS = value*1000;
  else if (index == 85) data.ERROR_REJECTION_VEL = value;
  else if (index == 86) data.ERROR_REJECTION_STD = value;
  else if (index == 87) data.ERROR_REJECTION_DT = value;
  else if (index == 90) data.BB_LAT1 = value;
  else if (index == 91) data.BB_LAT2 = value;
  else if (index == 92) data.BB_LON1 = value;
  else if (index == 93) data.BB_LON2 = value;
  else if (index == 94) data.CUBA_MAX_SECONDS = value;
  else if (index == 95) {
    data.TIMED_CUTDOWN_MILLIS = value;
    data.TIMED_CUTDOWN_ENABLE = true;
  }
  else if (index == 96) data.DLDT_SCALE = value;                          // Scalar factor on predicted DLDT for sunset 
  else if (index == 97) data.LAT_GPS_MANUAL = value;                      // Manual Latitude for sunset prediction
  else if (index == 98) data.LONG_GPS_MANUAL = value;                     // Manual Longitude for sunset precitions
  else if (index == 99) data.GPS_MANUAL_MODE = value;                     // Toggels if manual input or gps-recieved coords are used for sunset prediction
  else if (index == 100) data.GPS_MANUAL_MODE_OVERRIDE = value;           // Toggels if new getting new GPS coords trigger disabling of manual mode or not
  else if (index == 101) data.DEADMAN_ENABLED = (bool)value;
  else if (index == 102) data.DEADMAN_TIME = value * 1000. * 60 * 60; // hours
  else if (index == 123) data.RB_COOLDOWN = value * 1000;
  else if (index == 124) data.SWITCH_TO_MANUAL = (bool)value;
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
  data.BMP_ENABLE[0] = command & 0b0001;
  data.BMP_ENABLE[1] = command & 0b0010;
  data.BMP_ENABLE[2] = command & 0b0100;
  data.BMP_ENABLE[3] = command & 0b1000;
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
