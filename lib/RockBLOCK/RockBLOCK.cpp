/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu

  File: RockBlock.cpp
  --------------------------
  Implimentation of RockBlock.h
*/

#include "RockBLOCK.h"

void ISBDConsoleCallback(IridiumSBD *device, char c) {
  Serial.print(c);
}

void ISBDDiagCallback(IridiumSBD *device, char c) {
  Serial.print(c);
}

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the RockBlock module.
 */
bool RockBLOCK::init(bool shouldStartup) {
  bool success = false;
  pinMode(RB_GATE, OUTPUT);
  digitalWrite(RB_GATE, LOW);
  delay(2500);
	Serial.println("beginning rockblock");
  isbd.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  Serial1.begin(RB_BAUD);
  if (shouldStartup) {
    restart();
    success = true;
  }
  success = true; // lmao
  return success;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: restart
 * -------------------
 * This function restarts the RockBLOCK.
 */
void RockBLOCK::restart() {
  for (int i=0; i<3; i++) Serial.println();
  Serial.println("Starting up RB");
  EEPROM.write(EEPROMAddress, false);

  // is this necessary? dunno. let the jank flow~
  isbd.reentrant = false;
  isbd.asleep = true;
  isbd.remainingMessages = -1;
  isbd.nextChar = -1;
  isbd.remainingMessages = -1;
  isbd.msstmWorkaroundRequested = true;

  digitalWrite(RB_GATE, HIGH);

  Serial1.begin(RB_BAUD);

  wake();
  delay(1000);
  EEPROM.write(EEPROMAddress, true);
}

/*
 * Function: shutdown
 * -------------------
 * This function shuts down the RockBLOCK.
 */
void RockBLOCK::shutdown() {
  Serial.println("shutting down");
  //isbd.sleep();
  digitalWrite(RB_GATE, LOW);
  EEPROM.write(EEPROMAddress, false);
}

/*
 * Function: wake
 * -------------------
 * This function wakes up the RockBLOCK.
 */
bool RockBLOCK::wake() {
  uint8_t ret = isbd.begin();
  if(ret != ISBD_SUCCESS) {
    Serial.print("isbd.begin() failed with error code ");
    Serial.print(ret);
    Serial.print('\n');
    return false;
  }
  return true;
}

/*
 * Function: snooze
 * -------------------
 * This function sleeps the RockBLOCK.
 */
bool RockBLOCK::snooze() {
  uint8_t ret = isbd.sleep();
  if(ret != ISBD_SUCCESS) {
    Serial.print("isbd.sleep() failed with error code ");
    Serial.print(ret);
    Serial.print('\n');
    return false;
  }
  return true;
}

/*
 * Function: writeRead
 * -------------------
 * This function writes a bitstream across the communication interface.
 * It returns the length of a read message.
 */
int16_t RockBLOCK::writeRead(char* buff, uint16_t len) {
  if(len > BUFFER_SIZE) return -1;
  if(len < 0) return -1;
  size_t  bufferSize = sizeof(rxBuffer);
  write(buff, len);
  Serial.println("time to like send stuff");
  int weAlwaysCheckReturnCodes = isbd.sendReceiveSBDBinary(rxBuffer, len, rxBuffer, bufferSize);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("RETURN CODE");
  Serial.println(weAlwaysCheckReturnCodes);
  if(weAlwaysCheckReturnCodes != ISBD_SUCCESS) return -1;
  read(buff, bufferSize);
  return bufferSize;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: write
 * -------------------
 * This function writes a bitstream to the rockblock buffer.
 */
void RockBLOCK::write(char* buff, uint16_t len) {
  for(size_t i = 0; i < len; i++) {
    rxBuffer[i] = buff[i];
  }
}

/*
 * Function: read
 * -------------------
 * This function reads a bitstream from the rockblock buffer.
 */
void RockBLOCK::read(char* buff, uint16_t len) {
  for(size_t i = 0; i < len; i++) {
    buff[i] = rxBuffer[i];
  }
}
