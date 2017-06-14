/*
  Stanford Student Space Initiative
  Balloons | VALBAL | June 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu

  File: GPS.cpp
  --------------------------
  Implimentation of GPS.h
*/

#include "GPS.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the Ublox NEO-M8Q GPS module.
 */
bool GPS::init(bool shouldStartup) {
  bool success = false;
  pinMode(GPS_ENABLE_PIN, OUTPUT);
  digitalWrite(GPS_ENABLE_PIN, LOW);
  delay(2000);
  Serial1.begin(GPS_BAUD);
  if (shouldStartup) {
    success = restart();
  }
  return success;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: restart
 * -------------------
 * This function restarts the GPS.
 */
bool GPS::restart() {
  bool success = false;
  EEPROM.write(EEPROMAddress, false);
  digitalWrite(GPS_ENABLE_PIN, HIGH);
  delay(1000);
  EEPROM.write(EEPROMAddress, true);
  delay(3000);
  // success = setGPSMode(GPSOnlyExtra, sizeof(GPSOnlyExtra)/sizeof(uint8_t), GPS_LOCK_TIME);
  // success = setGPSMode(powerSave, sizeof(powerSave)/sizeof(uint8_t), GPS_LOCK_TIME);
  // success = setGPSMode(powerInterval, sizeof(powerInterval)/sizeof(uint8_t), GPS_LOCK_TIME);
  // success = setGPSMode(setTenthHz, sizeof(setTenthHz)/sizeof(uint8_t), GPS_LOCK_TIME);
  success = setGPSMode(flightMode, sizeof(flightMode)/sizeof(uint8_t), GPS_LOCK_TIME);
  return success;
}

/*
 * Function: hotstart
 * -------------------
 * This function hotstarts the GPS.
 */
void GPS::hotstart() {
  Serial1.println("$PUBX,00*33");
  delay(1000);
}

/*
 * Function: shutdown
 * -------------------
 * This function shutsdown the GPS.
 */
void GPS::shutdown() {
  digitalWrite(GPS_ENABLE_PIN, LOW);
  EEPROM.write(EEPROMAddress, false);
}

/*
 * Function: getLatitude
 * -------------------
 * This function returns the current latitude.
 */
float GPS::getLatitude() {
  return tinygps.location.lat();
}

/*
 * Function: getLongitude
 * -------------------
 * This function returns the current longitude.
 */
float GPS::getLongitude() {
  return tinygps.location.lng();
}

/*
 * Function: getAltitude
 * -------------------
 * This function returns the current altitude in meters.
 */
float GPS::getAltitude() {
  return tinygps.altitude.meters();
}

/*
 * Function: getSpeed
 * -------------------
 * This function returns the current speed in mph.
 */
float GPS::getSpeed() {
  return tinygps.speed.mph();
}

/*
 * Function: getCourse
 * -------------------
 * This function returns the current heading in degrees.
 */
float GPS::getCourse() {
  return tinygps.course.deg();
}

/*
 * Function: getSats
 * -------------------
 * This function returns the number of satelites detected.
 */
uint8_t GPS::getSats() {
  return tinygps.satellites.value();
}

/*
 * Function: smartDelay
 * -------------------
 * This function pauses the main thread while
 * still communicating with the comms interface.
 */
void GPS::smartDelay(uint32_t ms) {
  uint32_t startt = millis();
  do {
    while (Serial1.available()) {
      char c = Serial1.read();
      tinygps.encode(c);
    }
  } while (millis() - startt < ms);
}

/*********************************  HELPERS  **********************************/
bool GPS::setGPSMode(uint8_t* MSG, uint8_t len, uint16_t GPS_LOCK_TIME){
  Serial.println("Setting uBlox mode: ");
  uint32_t startTime = millis();
  uint8_t gps_set_sucess = 0;
  while(!gps_set_sucess) {
    if(millis() - startTime > GPS_TIMEOUT_TIME) return false;
    sendUBX(MSG, len);
    gps_set_sucess = getUBX_ACK(MSG);
  }
  smartDelay(GPS_LOCK_TIME);
  return true;
}

/*
 * Function: sendUBX
 * -------------------
 * This function sends a byte array of UBX protocol to the GPS.
 */
void GPS::sendUBX(uint8_t* MSG, uint8_t len) {
  for(int i = 0; i < len; i++) {
    Serial1.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  Serial1.println();
}

/*
 * Function: getUBX_ACK
 * -------------------
 * This function calculates the expected UBX ACK packet and parses the UBX response from GPS.
 */
bool GPS::getUBX_ACK(uint8_t* MSG) {
  uint8_t  b;
  uint8_t  ackByteID = 0;
  uint8_t  ackPacket[10];
  uint32_t startTime = millis();
  Serial.print(" * Reading ACK response: ");

  ackPacket[0] = 0xB5;	 // header
  ackPacket[1] = 0x62;	 // header
  ackPacket[2] = 0x05;	 // class
  ackPacket[3] = 0x01;	 // id
  ackPacket[4] = 0x02;	 // length
  ackPacket[5] = 0x00;   // space
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0;      // CK_A
  ackPacket[9] = 0;      // CK_B

  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (millis() - startTime < 3000) {
    if (ackByteID > 9) {
      Serial.println(" (SUCCESS!)");
      return true;
    }
    if (Serial1.available()) {
      b = Serial1.read();
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        Serial.print(b, HEX);
      }
      else ackByteID = 0;
    }
  }
  Serial.println(" (FAILED!)");
  return false;
}
