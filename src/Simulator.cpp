/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
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
  simulated.RAW_PRESSURE_1 = atof(values[0]);
  simulated.RAW_PRESSURE_2 = atof(values[1]);
  simulated.RAW_PRESSURE_3 = atof(values[2]);
  simulated.RAW_PRESSURE_4 = atof(values[3]);

  simulated.BMP_1_ENABLE = atof(values[4]);
  simulated.BMP_2_ENABLE = atof(values[5]);
  simulated.BMP_3_ENABLE = atof(values[6]);
  simulated.BMP_4_ENABLE = atof(values[7]);

  simulated.ALTITUDE = atof(values[8]);
  simulated.ASCENT_RATE = atof(values[9]);

  simulated.PRESS_BASELINE = atof(values[10]);
  // simulated.INCENTIVE_THRESHOLD = atof(values[11]);
  // simulated.RE_ARM_CONSTANT = atof(values[12]);
  simulated.BALLAST_ARM_ALT = atof(values[13]);

  simulated.VALVE_SETPOINT = atof(values[14]);
  simulated.VALVE_DURATION = atof(values[15]);
  simulated.VALVE_ALT_LAST = atof(values[16]);
  simulated.VALVE_VELOCITY_CONSTANT = atof(values[17]);
  simulated.VALVE_ALTITUDE_DIFF_CONSTANT = atof(values[18]);
  simulated.VALVE_LAST_ACTION_CONSTANT = atof(values[19]);
  simulated.BALLAST_SETPOINT = atof(values[20]);
  simulated.BALLAST_DURATION = atof(values[21]);
  simulated.BALLAST_ALT_LAST = atof(values[22]);
  simulated.BALLAST_VELOCITY_CONSTANT = atof(values[23]);
  simulated.BALLAST_ALTITUDE_DIFF_CONSTANT = atof(values[24]);
  simulated.BALLAST_LAST_ACTION_CONSTANT = atof(values[25]);

  // simulated.MANUAL_MODE = atof(values[26]);

  simulated.VALVE_INCENTIVE = atof(values[27]);
  simulated.BALLAST_INCENTIVE = atof(values[28]);
  // simulated.VALVE_STATE = atof(values[29]);
  // simulated.BALLAST_STATE = atof(values[30]);
  // simulated.VALVE_QUEUE = atof(values[31]);
  // simulated.BALLAST_QUEUE = atof(values[32]);
  simulated.NUM_VALVES = atof(values[33]);
  simulated.NUM_BALLASTS = atof(values[34]);
  // simulated.NUM_VALVE_ATTEMPTS = atof(values[35]);
  // simulated.NUM_BALLAST_ATTEMPTS = atof(values[36]);
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
