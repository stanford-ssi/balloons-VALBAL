/*
  Stanford Student Space Initiative
  Balloons | VALBAL | April 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu

  File: RockBlock.cpp
  --------------------------
  Implimentation of RockBlock.h
*/

#include "RockBLOCK.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the RockBlock module.
 */
bool RockBLOCK::init(bool shouldStartup, bool sleep) {
  bool success = false;
  pinMode(RB_GATE, OUTPUT);
  digitalWrite(RB_GATE, LOW);
  delay(2000);
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  Serial3.begin(RB_BAUD);
  if (shouldStartup) {
    restart(sleep);
    success = true;
  }
  return success;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: restart
 * -------------------
 * This function restarts the RockBLOCK.
 */
void RockBLOCK::restart(bool sleep) {
  EEPROM.write(EEPROMAddress, false);
  digitalWrite(RB_GATE, HIGH);
  delay(1000);
  wake();
  delay(3000);
  if(sleep) {
    snooze();
    delay(1000);
  }
  EEPROM.write(EEPROMAddress, true);
}

/*
 * Function: shutdown
 * -------------------
 * This function shuts down the RockBLOCK.
 */
void RockBLOCK::shutdown() {
  digitalWrite(RB_GATE, LOW);
  EEPROM.write(EEPROMAddress, false);
  isbd.begin();
}

/*
 * Function: wake
 * -------------------
 * This function wakes up the RockBLOCK.
 */
bool RockBLOCK::wake() {
  uint8_t ret = isbd.begin();
  if(ret != ISBD_SUCCESS) {
    failureWakeCount++;
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
    failureSleepCount++;
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
int16_t RockBLOCK::writeRead(char* buff, uint16_t len, bool sleep) {
  if(len > BUFFER_SIZE) return -1;
  if(len < 0) return -1;
  if(sleep && !wake()) return -1;
  size_t  bufferSize = sizeof(rxBuffer);
  write(buff, len);
  if(isbd.sendReceiveSBDBinary(rxBuffer, len, rxBuffer, bufferSize) != ISBD_SUCCESS) {
    if(sleep) snooze();
    return -1;
  }
  read(buff, bufferSize);
  if(sleep && !snooze()) return -1;
  return bufferSize;
}

/*
 * Function: getNumWake
 * -------------------
 * This function returns the number of failures to wake.
 */
uint32_t RockBLOCK::getNumWakeFailures(){
  return failureWakeCount;
}

/*
 * Function: getNumSleepFailures
 * -------------------
 * This function returns the number of failures to sleep.
 */
uint32_t RockBLOCK::getNumSleepFailures(){
  return failureSleepCount;
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
