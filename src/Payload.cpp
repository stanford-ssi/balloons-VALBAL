/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu

  File: Payload.cpp
  --------------------------
  Implementation of Payload.h
*/

#include "Payload.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the Payload object.
 */
bool Payload::init(bool shouldStartup) {
  bool success = false;
  if (shouldStartup) {
    restart();
    success = true;
  }
  return success;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: restart
 * -------------------
 * This function restarts the Payload.
 */
void Payload::restart() {
  EEPROM.write(EEPROMAddress, false);
  pinMode(payloadGate, OUTPUT);
  digitalWrite(payloadGate, HIGH);
  delay(1000);
  EEPROM.write(EEPROMAddress, true);
  Serial2.begin(115200);
}

/*
 * Function: shutdown
 * -------------------
 * This function shuts down the Payload.
 */
void Payload::shutdown() {
  digitalWrite(payloadGate, LOW);
  EEPROM.write(EEPROMAddress, false);
}

/*
 * Function: querrySensors
 * -------------------
 * This function gets the updated data stream from the payload.
 */
void Payload::querrySensors() {
  for(size_t i = 0; i < 100; i++) buf[i] = 0;
  digitalWrite(payloadGate, HIGH);
  if(!Serial2.available()) return;
  Serial2.setTimeout(10);
  Serial2.readBytesUntil('\n', buf, 100);
  if (strncmp (buf,"[LOG]",4) != 0) {
    char *tok = strtok(buf, ",");
    for(size_t i = 0; i < 3; i++) {
      values[i] = atof(tok);
      tok = strtok(NULL, ",");
    }
  }
}

/*
 * Function: getEuler
 * -------------------
 * This function returns the specified Euler value.
 */
float Payload::getEuler(uint8_t axis) {
  if(axis == 0) return values[0];
  if(axis == 1) return values[1];
  if(axis == 2) return values[2];
  return -1;
}
