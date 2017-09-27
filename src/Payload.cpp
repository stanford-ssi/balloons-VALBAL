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



int previous_heartbeat = LOW;


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
  pinMode(payloadGPIO1, INPUT);
  pinMode(payloadGPIO2, INPUT);
  return success;
}


bool Payload::send_message(vb_rf_message* msg) {
  bool payloadReady = digitalRead(payloadGPIO1);
  if (!payloadReady) return false;
  Serial2.write(RADIO_START_SEQUENCE, 4);
  int len = sizeof(vb_rf_message);
  for (int i = 0; i < sizeof(vb_rf_message); i++) {
    if (((uint8_t*)msg)[i] != 0)  {
      len = i + 1;
    }
  }
  Serial2.write((uint8_t*)msg, len);
  Serial2.write(RADIO_END_SEQUENCE, 4);
  return true;
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
  Serial.println("Got new config of size");
  Serial.println(len);
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
  //payloadReady = true;
  if (hasNewConfig) {
    Serial.println("sending new config");
    sendConfig();
  }
  sendDataFrame();
  sendHeartBeat();
  return true;
}

/*********************************  HELPERS  **********************************/


// Stack overflow ayy
int char2int(char input) {
  if(input >= '0' && input <= '9')
    return input - '0';
  if(input >= 'A' && input <= 'F')
    return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
    return input - 'a' + 10;
  return 0;
}

/*
 * Function: sendConfig
 * -------------------
 * This function sends the config over UART.
 */
bool Payload::sendConfig() {
  Serial.println("[PAYLOAD] Sending updated config to radio board.");

  const char* hex = SATCOMMS_BUFFER;
  vb_rf_message send_msg;

  send_msg.type = SET_CONFIG;
  int i = 0;
  while(*hex && hex[1]) {
    send_msg.data[i] = char2int(*hex)*16 + char2int(hex[1]);
    hex += 2;
    i++;
  }
  if (send_message(&send_msg)) {
    hasNewConfig = false;
    return true;
  } else return false;
}

/*
 * Function: sendDataFrame
 * -------------------
 * This function sends the dataframe over UART.
 */
bool Payload::sendDataFrame() {
  Serial.print("[PAYLOAD] Sending dataframe over, ");
  Serial.print(lengthBytes);
  Serial.println(" bytes.");
  vb_rf_message send_msg;
  send_msg.type = DATA_FRAME;
  send_msg.data[0] = lengthBytes;
  memcpy(send_msg.data+1, &theLatitude, 4);
  memcpy(send_msg.data+5, &theLongitude, 4);
  /**((float*)send_msg.data + 1) = theLatitude;
  *((float*)send_msg.data + 5) = theLongitude;*/
  //Serial.println("set latitude to");
  //Serial.print(*((float*)send_msg.data + 1));
  memcpy(send_msg.data + 9, DATA_BUFFER, lengthBytes);
  send_message(&send_msg);

  return true;
}

int heartBeatViolations = 0;

/*
 * Function: sendHeartBeat
 * -------------------
 * This function checks the status of the payload using a heartbeat.x
 */
bool Payload::sendHeartBeat() {
  bool payloadReady = digitalRead(payloadGPIO1);
  if (!payloadReady) { return true; }

  int beat = digitalRead(payloadGPIO2);
  if (previous_heartbeat != (int)digitalRead(GPIO2)) {
    Serial.println("[PAYLOAD] SOMETHING IS ROTTEN, heartbeat did not get a correct reply.");
    heartBeatViolations++;
  } else {
    //Serial.print("[PAYLOAD] Heartbeat OK: ");
    //Serial.println(previous_heartbeat, DEC);
  }
  Serial.print(heartBeatViolations);
  Serial.println(" heartbeat fuckups");
  vb_rf_message send_msg;
  send_msg.type = HEARTBEAT;
  previous_heartbeat = random(0, 2);
  send_msg.data[0] = previous_heartbeat;
  send_message(&send_msg);
  return true;
}
