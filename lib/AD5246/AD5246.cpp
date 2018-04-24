/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu

  File: AD5246.cpp
  --------------------------
  Implimentation of AD5246.h
*/

#include "AD5246.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the AD5246 resistor.
 */
bool AD5246::init() {
  setResistance(0x10);
  delay(100);
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: setResistance
 * -------------------
 * This function sets the resistor to the specified hex value.
 */
bool AD5246::setResistance(uint8_t hex) {
  // TODO actually use the right hex
  Serial.println("setting resistance fam");

  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  digitalWriteFast(18, HIGH);
  digitalWriteFast(19, HIGH);

    Serial.println("setting resistance fam");
    int mx = 128;

    // SDA 18
    // SCL 19
    bool stuffs[] = {0,1,0,1,1,1,0,0};
    digitalWriteFast(18, LOW);
    delayMicroseconds(10);
    for (int i=0; i<8; i++) {
      digitalWriteFast(19, LOW);
      digitalWriteFast(18, stuffs[i]);
      delayMicroseconds(10);
      digitalWriteFast(19, HIGH);
      delayMicroseconds(10);
      mx >>= 1;
    }
    digitalWriteFast(19, LOW);
    pinMode(18, INPUT);
    delayMicroseconds(10);
    digitalWriteFast(19, HIGH);
    delayMicroseconds(3);
    int stuff = digitalReadFast(18);
    if (stuff != 0) {
      Serial.println("NACK");
    }
    pinMode(18, OUTPUT);
    delayMicroseconds(7);

    bool stuffs2[] = {hex & 128,hex & 64,hex & 32,hex & 16,hex & 8,hex & 4,hex & 2,hex & 1};
    for (int i=0; i<8; i++) {
      digitalWriteFast(19, LOW);
      digitalWriteFast(18, stuffs2[i]);
      delayMicroseconds(10);
      digitalWriteFast(19, HIGH);
      delayMicroseconds(10);
      mx >>= 1;
    }
    digitalWriteFast(19, LOW);
    pinMode(18, INPUT);
    delayMicroseconds(10);
    digitalWriteFast(19, HIGH);
    delayMicroseconds(3);
    int stuff2 = digitalReadFast(18);
    if (stuff2 != 0) {
      Serial.println("NACK2");
    }
    pinMode(18, OUTPUT);
    delayMicroseconds(7);
    digitalWriteFast(19, LOW);
    delayMicroseconds(10);
    digitalWriteFast(19, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(18, HIGH);
  return true;
}
