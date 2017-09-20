/*
  Stanford Student Space Initiative
  Balloons | VALBAL | June 2017
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
  bool success = false;
  CSV = SD.open("HITL.txt");
  if(CSV) success = true;
  getLine();
  return success;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: readData
 * -------------------
 * This function parses the simulated data provided by the model.
 */
DataFrame Simulator::readData() {
  getLine();
  size_t i = 0;
  char *tok = strtok(buffer, ",");
  while (tok != NULL) {
    values[i++] = atof(tok);
    tok = strtok(NULL, ",");
  }
  simulated.RAW_PRESSURE_1                 = values[0];
  simulated.RAW_PRESSURE_2                 = values[1];
  simulated.RAW_PRESSURE_3                 = values[2];
  simulated.RAW_PRESSURE_4                 = values[3];
  simulated.BMP_1_ENABLE                   = values[4];
  simulated.BMP_2_ENABLE                   = values[5];
  simulated.BMP_3_ENABLE                   = values[6];
  simulated.BMP_4_ENABLE                   = values[7];
  simulated.ALTITUDE_BAROMETER             = values[8];
  simulated.ASCENT_RATE                    = values[9];
  simulated.PRESS_BASELINE                 = values[10];
  simulated.BALLAST_ARM_ALT                = values[11];
  simulated.VALVE_SETPOINT                 = values[12];
  simulated.VALVE_VENT_DURATION            = values[13];
  simulated.VALVE_ALT_LAST                 = values[14];
  simulated.VALVE_VELOCITY_CONSTANT        = values[15];
  simulated.VALVE_ALTITUDE_DIFF_CONSTANT   = values[16];
  simulated.VALVE_LAST_ACTION_CONSTANT     = values[17];
  simulated.BALLAST_SETPOINT               = values[18];
  simulated.BALLAST_DROP_DURATION          = values[19];
  simulated.BALLAST_ALT_LAST               = values[20];
  simulated.BALLAST_VELOCITY_CONSTANT      = values[21];
  simulated.BALLAST_ALTITUDE_DIFF_CONSTANT = values[22];
  simulated.BALLAST_LAST_ACTION_CONSTANT   = values[23];
  simulated.VALVE_INCENTIVE                = values[24];
  simulated.BALLAST_INCENTIVE              = values[25];
  simulated.VALVE_NUM_ACTIONS              = values[26];
  simulated.BALLAST_NUM_ACTIONS            = values[27];
  loopInterval                             = values[28];
  // simulated.INCENTIVE_THRESHOLD            = values[28];
  // simulated.RE_ARM_CONSTANT                = values[29];
  // simulated.MANUAL_MODE                    = values[30];
  // simulated.VALVE_STATE                    = values[31];
  // simulated.BALLAST_STATE                  = values[32];
  // simulated.VALVE_QUEUE                    = values[33];
  // simulated.BALLAST_QUEUE                  = values[34];
  // simulated.VALVE_NUM_ATTEMPTS             = values[45];
  // simulated.BALLAST_NUM_ATTEMPTS           = values[36];
  return simulated;
}

uint32_t Simulator::getLoopTime() {
  return loopInterval;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: getLine
 * -------------------
 * This function reads a line from the simulated data.
 */
void Simulator::getLine() {
  for (size_t i = 0; i < UART_BUFFER_SIZE; i++) buffer[i] = 0;
  size_t i = 0;
  CSV.seek(pos);
  while (CSV.available() && i < UART_BUFFER_SIZE) {
    char c = CSV.read();
    if(c == '\n') {
      pos = CSV.position();
      return;
    }
    buffer[i] = c;
    i++;
  }
  pos = CSV.position();
  return;
}
