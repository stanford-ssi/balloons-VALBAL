/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

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
  pinMode(payloadGPIO1, INPUT);
  pinMode(payloadGPIO2, INPUT);
  pinMode(payloadDAC, OUTPUT);
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
 * Function: sendConfig
 * -------------------
 * This function sets a config message from the avionics.
 */
bool Payload::setConfig(const char * str, size_t len){
  if (len >= SATCOMMS_BUFFER_SIZE) return false;
  for (uint16_t i = 0; i < SATCOMMS_BUFFER_SIZE; i++) SATCOMMS_BUFFER[i] = 0;
  for (uint16_t i = 0; i < len; i++) SATCOMMS_BUFFER[i] = str[i];
  hasNewConfig = true;
  return true;
}

/*
 * Function: readyDataFrame
 * -------------------
 * This function readies a data frame message from the avionics.
 */
bool Payload::readyDataFrame(){
  lengthBits  = 0;
  lengthBytes = 0;
  for(uint16_t i = 0; i < DATA_BUFFER_SIZE; i++) DATA_BUFFER[i] = 0;
  return true;
}
/*
 * Function: addVariable
 * -------------------
 * This function compresses a single variable into a scaled digital bitmask.
 */
bool Payload::addVariable(float var, float minimum, float maximum, int16_t resolution) {
  if (resolution <= 0) return false;
  if (var < minimum) var = minimum;
  if (var > maximum) var = maximum;
  int32_t adc = round( (pow(2, resolution) - 1) * (var - minimum) / (maximum - minimum));
  int16_t byteIndex = lengthBits / 8;
  int16_t bitIndex = 7 - (lengthBits % 8);
  for (int16_t i = resolution - 1; i >= 0; i--) {
    bool bit = adc & (1 << i);
    if (bit) DATA_BUFFER[byteIndex] |= (1 << bitIndex);
    bitIndex -= 1;
    if (bitIndex < 0) {
      bitIndex = 7;
      byteIndex++;
    }
  }
  lengthBits += resolution;
}

/*
 * Function: setDataFrame
 * -------------------
 * This function sets a data frame message from the avionics.
 */
bool Payload::setDataFrame(){
  lengthBits += 8 - (lengthBits % 8);
  lengthBytes = lengthBits / 8;
  return true;
}

/*
 * Function: run
 * -------------------
 * This function handles the hardware interface and timing with the payload.
 */
bool Payload::run(){
  bool payloadReady = digitalRead(payloadGPIO1);
  if (payloadReady) {
    if (hasNewConfig) {
      sendConfig();
      hasNewConfig = false;
    }
    sendDataFrame();
    sendHeartBeat();
  }
  return true;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: sendConfig
 * -------------------
 * This function sends the config over UART.
 */
bool Payload::sendConfig() {
  //send SATCOMMS_BUFFER over UART
  return true;
}

/*
 * Function: sendDataFrame
 * -------------------
 * This function sends the dataframe over UART.
 */
bool Payload::sendDataFrame() {
  //send DATA_BUFFER over UART
  return true;
}

/*
 * Function: sendHeartBeat
 * -------------------
 * This function checks the status of the payload using a heartbeat.
 */
bool Payload::sendHeartBeat() {
  bool heartbeat = digitalRead(payloadGPIO2);
  //send heartbeat over UART
  return true;
}
