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
  CSV = SD.open("LOGGER07.txt");
  // myFile = SD.open("ssi-48-sd_selected.csv");
  if(CSV) success = true;
  getLine();
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
  simulated.ALTITUDE                       = values[8];
  simulated.ASCENT_RATE                    = values[9];
  simulated.PRESS_BASELINE                 = values[10];
  simulated.INCENTIVE_THRESHOLD            = values[11];
  simulated.RE_ARM_CONSTANT                = values[12];
  simulated.BALLAST_ARM_ALT                = values[13];
  simulated.VALVE_SETPOINT                 = values[14];
  simulated.VALVE_DURATION                 = values[15];
  simulated.VALVE_ALT_LAST                 = values[16];
  simulated.VALVE_VELOCITY_CONSTANT        = values[17];
  simulated.VALVE_ALTITUDE_DIFF_CONSTANT   = values[18];
  simulated.VALVE_LAST_ACTION_CONSTANT     = values[19];
  simulated.BALLAST_SETPOINT               = values[20];
  simulated.BALLAST_DURATION               = values[21];
  simulated.BALLAST_ALT_LAST               = values[22];
  simulated.BALLAST_VELOCITY_CONSTANT      = values[23];
  simulated.BALLAST_ALTITUDE_DIFF_CONSTANT = values[24];
  simulated.BALLAST_LAST_ACTION_CONSTANT   = values[25];
  simulated.MANUAL_MODE                    = values[26];
  simulated.VALVE_INCENTIVE                = values[27];
  simulated.BALLAST_INCENTIVE              = values[28];
  simulated.VALVE_STATE                    = values[29];
  simulated.BALLAST_STATE                  = values[30];
  simulated.VALVE_QUEUE                    = values[31];
  simulated.BALLAST_QUEUE                  = values[32];
  simulated.NUM_VALVES                     = values[33];
  simulated.NUM_BALLASTS                   = values[34];
  simulated.NUM_VALVE_ATTEMPTS             = values[35];
  simulated.NUM_BALLAST_ATTEMPTS           = values[36];
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
