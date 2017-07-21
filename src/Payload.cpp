/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
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
  pinMode(payloadGate, OUTPUT);
  pinMode(payloadGPIO1, OUTPUT);
  pinMode(payloadGPIO1, OUTPUT);
  digitalWrite(payloadGate, HIGH);
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
  digitalWrite(payloadGate, LOW);
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
  digitalWrite(payloadGate, HIGH);
  EEPROM.write(EEPROMAddress, false);
}

/*
 * Function: getMessage
 * -------------------
 * This function gets the data stream from the payload.
 */
size_t Payload::getMessage() {
  for(size_t i = 0; i < 100; i++) buf[i] = 0;
  digitalWrite(payloadGate, LOW);
  if(!Serial2.available()) return 0;
  Serial2.setTimeout(10);
  size_t numBytes = Serial2.readBytesUntil('\n', buf, 100);
  if (strncmp (buf,"[LOG]",4) != 0) return numBytes;
  return 0;
}
