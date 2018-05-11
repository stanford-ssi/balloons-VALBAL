/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Radio.cpp
  --------------------------
  Implementation of Radio.h
*/

#include "Radio.h"



int previous_heartbeat = LOW;


bool Radio::init(bool shouldStartup) {
  bool success = false;
  pinMode(35, OUTPUT);
  digitalWrite(35, LOW);
  if (shouldStartup) {
    restart();
    success = true;
  }
  success = true;
  return success;
}


bool Radio::send_message(vb_rf_message* msg) {
  Serial6.write(RADIO_START_SEQUENCE, 4);
  int len = sizeof(vb_rf_message);
  for (int i = 0; i < sizeof(vb_rf_message); i++) {
    if (((uint8_t*)msg)[i] != 0)  {
      len = i + 1;
    }
  }
  Serial6.write((uint8_t*)msg, len);
  Serial6.write(RADIO_END_SEQUENCE, 4);
  return true;
}

void Radio::restart() {
  Serial.println("turning on that boi");
  pinMode(35, OUTPUT);
  digitalWrite(35, HIGH);
  Serial6.begin(VBRF_BAUD_RATE);
  lastStartupTime = millis();
  sent = false;
}

void Radio::shutdown() {
  pinMode(35, LOW);
}

bool Radio::readyDataFrame(){
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
bool Radio::addVariable(float var, float minimum, float maximum, int16_t resolution) {
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
bool Radio::setDataFrame(){
  lengthBits += 8 - (lengthBits % 8);
  lengthBytes = lengthBits / 8;
  return true;
}

/*
 * Function: run
 * -------------------
 * This function handles the hardware interface and timing with the payload.
 */
bool Radio::run(){
  sendDataFrame();
  return true;
}


/*
 * Function: sendDataFrame
 * -------------------
 * This function sends the dataframe over UART.
 */
bool Radio::sendDataFrame() {
  Serial.print("[RADIO] Sending dataframe over, ");
  Serial.print(lengthBytes);
  Serial.println(" bytes.");
  vb_rf_message send_msg;
  send_msg.data[0] = lengthBytes;
  memcpy(send_msg.data + 1, DATA_BUFFER, lengthBytes);
  send_message(&send_msg);

  return true;
}

