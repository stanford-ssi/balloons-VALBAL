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
  int jank_offset = 1;
  simulated.RAW_PRESSURE_1                 = values[0 + jank_offset];
  simulated.RAW_PRESSURE_2                 = values[1 + jank_offset];
  simulated.RAW_PRESSURE_3                 = values[2 + jank_offset];
  simulated.RAW_PRESSURE_4                 = values[3 + jank_offset];
  simulated.BMP_1_ENABLE                   = values[4 + jank_offset];
  simulated.BMP_2_ENABLE                   = values[5 + jank_offset];
  simulated.BMP_3_ENABLE                   = values[6 + jank_offset];
  simulated.BMP_4_ENABLE                   = values[7 + jank_offset];
  simulated.ALTITUDE                       = values[8 + jank_offset];
  simulated.ASCENT_RATE                    = values[9 + jank_offset];
  simulated.PRESS_BASELINE                 = values[10 + jank_offset];
  simulated.BALLAST_ARM_ALT                = values[11 + jank_offset];
  simulated.VALVE_SETPOINT                 = values[12 + jank_offset];
  simulated.VALVE_DURATION                 = values[13 + jank_offset];
  simulated.VALVE_ALT_LAST                 = values[14 + jank_offset];
  simulated.VALVE_VELOCITY_CONSTANT        = values[15 + jank_offset];
  simulated.VALVE_ALTITUDE_DIFF_CONSTANT   = values[16 + jank_offset];
  simulated.VALVE_LAST_ACTION_CONSTANT     = values[17 + jank_offset];
  simulated.BALLAST_SETPOINT               = values[18 + jank_offset];
  simulated.BALLAST_DURATION               = values[19 + jank_offset];
  simulated.BALLAST_ALT_LAST               = values[20 + jank_offset];
  simulated.BALLAST_VELOCITY_CONSTANT      = values[21 + jank_offset];
  simulated.BALLAST_ALTITUDE_DIFF_CONSTANT = values[22 + jank_offset];
  simulated.BALLAST_LAST_ACTION_CONSTANT   = values[23 + jank_offset];
  simulated.VALVE_INCENTIVE                = values[24 + jank_offset];
  simulated.BALLAST_INCENTIVE              = values[25 + jank_offset];
  simulated.NUM_VALVES                     = values[26 + jank_offset];
  simulated.NUM_BALLASTS                   = values[27 + jank_offset];
  // simulated.INCENTIVE_THRESHOLD            = values[28];
  // simulated.RE_ARM_CONSTANT                = values[29];
  // simulated.MANUAL_MODE                    = values[30];
  // simulated.VALVE_STATE                    = values[31];
  // simulated.BALLAST_STATE                  = values[32];
  // simulated.VALVE_QUEUE                    = values[33];
  // simulated.BALLAST_QUEUE                  = values[34];
  // simulated.NUM_VALVE_ATTEMPTS             = values[45];
  // simulated.NUM_BALLAST_ATTEMPTS           = values[36];
  return simulated;
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
