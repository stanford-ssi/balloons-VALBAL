/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2018
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Keegan Mehall | kmehall@stanford.edu

  File: Output.cpp
  --------------------------
  Implimentation of Output.h
*/

#include "Output.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the PCB hardware.
 */
void Output::init() {
  //from old hardware class
  analogReference(INTERNAL);
  analogReadResolution(12);
  wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  wire.setDefaultTimeout(5000);

  //from old avionics
  pinMode(VALVE_OPEN,    OUTPUT);
  pinMode(VALVE_CLOSE,    OUTPUT);
  pinMode(BALLAST_FORWARD,  OUTPUT);
  pinMode(BALLAST_REVERSE,  OUTPUT);
  pinMode(CUTDOWN,          OUTPUT);

  digitalWrite(CUTDOWN, LOW);
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: runLED
 * -------------------
 * This function turns the LED on or off.
 */
void Output::runLED(bool on) {
  digitalWrite(LED_PIN, HIGH);
}

/*
 * Function: EEPROMReadlong
 * -------------------
 * This function reads an int32_t from the given address.
 */
int32_t Output::EEPROMReadlong(uint8_t address) {
  int32_t four = EEPROM.read(address);
  int32_t three = EEPROM.read(address + 1);
  int32_t two = EEPROM.read(address + 2);
  int32_t one = EEPROM.read(address + 3);
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

/*
 * Function: EEPROMWritelong
 * -------------------
 * This function writes an int32_t to the given address.
 */
void Output::EEPROMWritelong(uint8_t address, int32_t value) {
  uint8_t four = (value & 0xFF);
  uint8_t three = ((value >> 8) & 0xFF);
  uint8_t two = ((value >> 16) & 0xFF);
  uint8_t one = ((value >> 24) & 0xFF);
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

void Output::openValve(uint16_t speed){
  analogWrite(VALVE_CLOSE, LOW);
  analogWrite(VALVE_OPEN,  speed);
}

void Output::closeValve(uint16_t speed){
  analogWrite(VALVE_CLOSE, speed);
  analogWrite(VALVE_OPEN,  LOW);
}

void Output::stopValve(){
  analogWrite(VALVE_CLOSE, LOW);
  analogWrite(VALVE_OPEN,  LOW);
}

//ballast:
void Output::runBallast(bool direction, uint16_t speed){
  if(direction){
    analogWrite(BALLAST_FORWARD, speed);
    analogWrite(BALLAST_REVERSE, LOW);
  }else{
    analogWrite(BALLAST_FORWARD, LOW);
    analogWrite(BALLAST_REVERSE, speed);
  }
}

void Output::stopBallast(){
  analogWrite(BALLAST_FORWARD, LOW);
  analogWrite(BALLAST_REVERSE, LOW);
};

//cutdown:
void Output::cutdown(){
  analogWrite(CUTDOWN, HIGH);
}

void Output::stopCutdown(){
  analogWrite(CUTDOWN, LOW);
}
