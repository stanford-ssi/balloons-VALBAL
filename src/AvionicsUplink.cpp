#include "Avionics.h"

/*
 * Function: parseCommand
 * -------------------
 * This function parses the command received from the RockBLOCK.
 */
void Avionics::parseCommand(int16_t len) {
  Serial.println("Got stuff");
  COMMS_BUFFER[len] = 0;
  if(COMMS_BUFFER[0]=='$') {
    Avionics::parseCommandNew(len);
    return;
  }
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

bool compareTime(uint32_t timestamp1, uint32_t timestamp2) {
    return (timestamp1<timestamp2);
}

/*
 * Function: parseCommandNew
 * -------------------
 * This function parses planned commands received from the RockBLOCK and places them in an array.
 */
void Avionics::parseCommandNew(int16_t len) {
    //for(uint8_t i = 0; i < PLANNED_COMMANDS_SIZE; i++) PLANNED_COMMANDS[i] = {-1, -1, ""};
    uint32_t plannedIndex = 0;
    uint8_t commandIndex;
    uint32_t time;
    uint8_t startIndex = 1;
    uint8_t endIndex = 1; // 0 is "P"
    bool interpolate = false;
    bool append = false;
    bool recent = false;
    bool readingFlags = true;
    while(endIndex < len) {
        if(startIndex==1 && readingFlags) {
            if(COMMS_BUFFER[endIndex]=='i') interpolate = true;
            else if(COMMS_BUFFER[endIndex]=='a') append = true;
            else if(COMMS_BUFFER[endIndex]=='r') recent = true;
            else {
                startIndex = endIndex;
                readingFlags = 0;
            }
        } else if(COMMS_BUFFER[endIndex]=='#') { // using '#' as deliminator between commandIndex and commands
            if(!append) { /* clear the PLANNED_COMMANDS */ }
            if(endIndex==startIndex) return;
            char commandIndexC[endIndex-startIndex+1];
            commandIndexC[endIndex-startIndex] = '\0';
            memcpy(commandIndexC, COMMS_BUFFER[startIndex], endIndex-startIndex);
            commandIndex = atoi(commandIndexC); // should stoi be used in case of error?
            if (commandIndex < 0 || commandIndex > 128) return;
            startIndex=endIndex+1;
        } else if(COMMS_BUFFER[endIndex]==':') { // a timestamp was just read
            if(endIndex==startIndex) return;
            char timestampC[endIndex-startIndex+1];
            timestampC[endIndex-startIndex] = '\0';
            memcpy(timestampC, COMMS_BUFFER[startIndex], endIndex-startIndex);
            timestamp = atoi(timestampC);
            startIndex=endIndex+1;
        } else if(COMMS_BUFFER[endIndex]==','
                  || COMMS_BUFFER[endIndex]=='&'
                  || (endIndex+1==len && startIndex!=endIndex)) { // command value paired with that time was just read
            if(endIndex==startIndex) return;
            char commandValueC[endIndex-startIndex+1];
            commandValueC[endIndex-startIndex] = '\0';
            memcpy(commandString, COMMS_BUFFER[startIndex], endIndex-startIndex);
            //PLANNED_COMMANDS[commandIndex][plannedIndex].COMMAND_INDEX = commandIndex;
            PLANNED_COMMANDS[commandIndex][plannedIndex].TIME = timestamp;
            memcpy(PLANNED_COMMANDS[commandIndex][plannedIndex].COMMAND_STRING, &commandString, strlen(commandString)+1);
            if(COMMS_BUFFER[endIndex]=='&') readingFlags = true;
            plannedIndex++;
            startIndex=endIndex+1;
        }
        endIndex++;
    }
    for(uint8_t i = 0; i<plannedIndex; i++) PLANNED_COMMANDS[commandIndex][plannedIndex].INTERPOLATE = interpolate;
    for(uint8_t i = 0; i<plannedIndex; i++) PLANNED_COMMANDS[commandIndex][plannedIndex].RECENT = recent;
    std::sort(PLANNED_COMMANDS[commandIndex][0],PLANNED_COMMANDS[commandIndex][PLANNED_COMMANDS_SIZE-1],compareTime);
}

/*
 * Function: updateConstant
 * -------------------
 * This function updates the state appropriate state variable
 * based on the command index.
 */

  extern int bal_duty;
	 extern int val_duty;

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
  else if (index == 17) val_duty = value; // Valve motor speed open | Between 0 and 255
  else if (index == 18) val_duty = value; // Valve motor speed close | Between 0 and 255
  else if (index == 19) bal_duty = value; // Ballast motor speed | Between 0 and 255
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
  else if (index == 33) data.CURRENT_CONTROLLER_INDEX = value; // Controller Index | 0: Legacy; 1: Lasagna
  else if (index == 56) data.LAS_CONSTANTS.gain                 =   value; // Total gain magnititude (g / m)
  else if (index == 57) data.LAS_CONSTANTS.damping              =   value; // damping ratio (unitless)
  else if (index == 58) data.LAS_CONSTANTS.v_gain               =   value; // velocity gain (g/s / m/s)
  else if (index == 59) data.LAS_CONSTANTS.h_gain               =   value; // altitude gain (m/s / km)
  else if (index == 60) data.LAS_CONSTANTS.bal_dldt             =   value; // balast dl/dt (g / s)
  else if (index == 61) data.LAS_CONSTANTS.val_dldt_a           =   value;
  else if (index == 62) data.LAS_CONSTANTS.val_dldt_b           =   value; // valve dl/dt (g / s))
  else if (index == 63) data.LAS_CONSTANTS.bal_tmin             =   value; // minimum ballast event time (s)
  else if (index == 64) data.LAS_CONSTANTS.val_tmin             =   value; // minimum valve event time (s)
  else if (index == 65) data.LAS_CONSTANTS.setpoint             =   value; // altidute comand (m)
  else if (index == 66) data.LAS_CONSTANTS.tolerance            =   value; // comand tollerance (m)
  else if (index == 67) data.LAS_CONSTANTS.k_drag               =   value; // drag constant, (m/s / g)
  else if (index == 68) data.LAS_CONSTANTS.kfuse_val            =   value; // scale factor on effect of valve actions (unitless)
  else if (index == 69) data.LAS_CONSTANTS.v_limit              =   value; // velocity limit commanded by altitude loop (m/s)
  else if (index == 70) data.LAS_CONSTANTS.equil_h_thresh       =   value; // altitude where controller transitions to normal mode (m)
  else if (index == 71) data.LAS_CONSTANTS.launch_h_thresh      =   value; // change in altitide required to detect launch (m)
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
  else if (index == 125) data.USE_ALTITUDE_CORRECTED = (bool)value;       // Select weather or not the controller should use ALTITUDE_CORRECTED
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
