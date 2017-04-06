/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
  Davy Ragland | dragland@stanford.edu

  File: Simulator.cpp
  --------------------------
  Implementation of Simulator.h
*/

#include "Simulator.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the Simulator objects.
 */
 bool Simulator::init() {
   bool success = true;
   return success;
 }

/********************************  FUNCTIONS  *********************************/
/*
 * Function: readData
 * -------------------
 * This function parses the simulated data provided by the model.
 */
DataFrame Simulator::readData() {
  size_t len = 0;
  while(len == 0) len = getLine();
  char *values[100];
  size_t i = 0;
  values[i] = strtok(buffer,",");
  while(values[i] != NULL) values[++i] = strtok(NULL,",");
  simulated.TIME = atof(values[0]);
  simulated.LAT_GPS = atof(values[1]);
  simulated.LONG_GPS = atof(values[2]);
  simulated.ALTITUDE = atof(values[3]);
  simulated.ALTITUDE_GPS = atof(values[4]);
  simulated.ASCENT_RATE = atof(values[5]);
  simulated.VALVE_INCENTIVE = atof(values[6]);
  simulated.BALLAST_INCENTIVE = atof(values[7]);
  simulated.VALVE_STATE = atof(values[8]);
  simulated.BALLAST_STATE = atof(values[9]);
  simulated.VALVE_QUEUE = atof(values[10]);
  simulated.BALLAST_QUEUE = atof(values[11]);
  simulated.NUM_VALVES = atof(values[12]);
  simulated.NUM_BALLASTS = atof(values[13]);
  simulated.NUM_VALVE_ATTEMPTS = atof(values[14]);
  simulated.NUM_BALLAST_ATTEMPTS = atof(values[15]);
  simulated.CUTDOWN_STATE = atof(values[16]);
  simulated.PRESS = atof(values[17]);
  simulated.TEMP = atof(values[18]);
  simulated.JOULES = atof(values[19]);
  simulated.VOLTAGE = atof(values[20]);
  simulated.CURRENT = atof(values[21]);
  simulated.CURRENT_GPS = atof(values[22]);
  simulated.CURRENT_RB = atof(values[23]);
  simulated.CURRENT_MOTORS = atof(values[24]);
  simulated.CURRENT_PAYLOAD = atof(values[25]);
  simulated.TEMP_NECK = atof(values[26]);
  simulated.SPEED_GPS = atof(values[27]);
  simulated.HEADING_GPS = atof(values[28]);
  simulated.NUM_SATS_GPS = atof(values[29]);
  simulated.LOOP_TIME = atof(values[30]);
  simulated.RB_SENT_COMMS = atof(values[31]);
  simulated.COMMS_INTERVAL = atof(values[32]);
  simulated.GPS_INTERVAL = atof(values[33]);
  simulated.TEMP_SETPOINT = atof(values[34]);
  simulated.MANUAL_MODE = atof(values[35]);
  simulated.RB_SHOULD_USE = atof(values[36]);
  simulated.GPS_SHOULD_USE = atof(values[37]);
  simulated.HEATER_SHOULD_USE = atof(values[38]);
  simulated.HEATER_STRONG_ENABLE = atof(values[39]);
  simulated.HEATER_WEEK_ENABLE = atof(values[40]);
  simulated.BAT_GOOD_STATE = atof(values[41]);
  simulated.CURR_GOOD_STATE = atof(values[42]);
  simulated.PRES_GOOD_STATE = atof(values[43]);
  simulated.TEMP_GOOD_STATE = atof(values[44]);
  simulated.RB_GOOD_STATE = atof(values[45]);
  simulated.GPS_GOOD_STATE = atof(values[46]);
  simulated.PRESS_BASELINE = atof(values[47]);
  simulated.INCENTIVE_NOISE = atof(values[48]);
  simulated.INCENTIVE_THRESHOLD = atof(values[49]);
  simulated.RE_ARM_CONSTANT = atof(values[50]);
  simulated.BALLAST_ARM_ALT = atof(values[51]);
  simulated.VALVE_SETPOINT = atof(values[52]);
  simulated.VALVE_DURATION = atof(values[53]);
  simulated.VALVE_ALT_LAST = atof(values[54]);
  simulated.VALVE_VELOCITY_CONSTANT = atof(values[55]);
  simulated.VALVE_ALTITUDE_DIFF_CONSTANT = atof(values[56]);
  simulated.VALVE_LAST_ACTION_CONSTANT = atof(values[57]);
  simulated.BALLAST_SETPOINT = atof(values[58]);
  simulated.BALLAST_DURATION = atof(values[59]);
  simulated.BALLAST_ALT_LAST = atof(values[60]);
  simulated.BALLAST_VELOCITY_CONSTANT = atof(values[61]);
  simulated.BALLAST_ALTITUDE_DIFF_CONSTANT = atof(values[61]);
  simulated.BALLAST_LAST_ACTION_CONSTANT = atof(values[62]);
  simulated.SHOULD_CUTDOWN = atof(values[63]);
  simulated.SHOULD_LED = atof(values[64]);
  simulated.SETUP_STATE = atof(values[65]);
  simulated.DEBUG_STATE = atof(values[66]);
  simulated.FORCE_VALVE = atof(values[67]);
  simulated.FORCE_BALLAST = atof(values[68]);
  simulated.REPORT_MODE = atof(values[69]);
  simulated.BMP_1_ENABLE = atof(values[70]);
  simulated.BMP_2_ENABLE = atof(values[71]);
  simulated.BMP_3_ENABLE = atof(values[72]);
  simulated.BMP_4_ENABLE = atof(values[73]);
  simulated.RAW_TEMP_1 = atof(values[74]);
  simulated.RAW_TEMP_2 = atof(values[75]);
  simulated.RAW_TEMP_3 = atof(values[76]);
  simulated.RAW_TEMP_4 = atof(values[77]);
  simulated.RAW_PRESSURE_1 = atof(values[78]);
  simulated.RAW_PRESSURE_2 = atof(values[79]);
  simulated.RAW_PRESSURE_3 = atof(values[80]);
  simulated.RAW_PRESSURE_4 = atof(values[81]);
  simulated.ALTITUDE_LAST = atof(values[82]);
  simulated.GPS_LAST = atof(values[83]);
  simulated.COMMS_LAST = atof(values[84]);
  simulated.COMMS_LENGTH = atof(values[85]);
  return simulated;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: getLine
 * -------------------
 * This function reads text across UART.
 */
size_t Simulator::getLine() {
  for (size_t i = 0; i < UART_BUFFER_SIZE; i++) buffer[i] = 0;
  size_t i = 0;
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\n') {
      return i;
    }
    buffer[i] = c;
    i++;
  }
}
