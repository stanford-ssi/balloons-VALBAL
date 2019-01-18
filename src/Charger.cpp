/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu

  File: Charger.cpp
  --------------------------
  Implimentation of Charger.h
*/

#include <SPI.h>

#include "Charger.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the PCB hardware.
 */
bool Charger::init() {
	Serial.println("hellooooooooooooooooooooooo!");
  pinMode(SUPER_CAP_ENABLE, OUTPUT);
  pinMode(FIVE_VOLT_ENABLE, OUTPUT);
  //digitalWrite(SUPER_CAP_ENABLE, LOW);
	digitalWrite(SUPER_CAP_ENABLE, HIGH);
  digitalWrite(FIVE_VOLT_ENABLE, LOW);
	pinMode(POT_CS, OUTPUT);
	SPI.begin();
	runChargerPID(0, 20);
  /*if (resistor.init()){
    chargingLimit = 3;
    digitalWrite(SUPER_CAP_ENABLE, HIGH);
    return true;
  }*/
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: enable5VBoost
 * -------------------
 * This function enables the 5 Volt line.
 */
void Charger::enable5VBoost() {
  digitalWrite(FIVE_VOLT_ENABLE, HIGH);
}

/*
 * Function: disable5VBoost
 * -------------------
 * This function disables the 5 Volt line.
 */
void Charger::disable5VBoost() {
  digitalWrite(FIVE_VOLT_ENABLE, LOW);
}

/*
 * Function: runCharger
 * -------------------
 * This function calcualtes and updates the desired charging output.
 */
void Charger::runChargerPID(uint8_t resistorMode, float temp) {
  uint8_t hex = 0x10;
  chargingLimit = resistorMode;
  if (resistorMode == 1) hex = 0x38;
  if (resistorMode == 2) hex = 0x23;
  if (resistorMode == 22) hex = 0x20;
  if (resistorMode == 3) hex = 0x10;
  if (resistorMode == 4) hex = 0x08;
  if (resistorMode == 0) {
    chargingLimit = 3;
    if (temp <= CHARGER_TEMP_THRESH_HIGH) {
      hex = 0x23;
      chargingLimit = 2;
    }
    if (temp <= CHARGER_TEMP_THRESH_LOW) {
      hex = 0x38;
      chargingLimit = 1;
    }
  }
	//hex = 0x10;
	//Serial.println("spi transfer ");
	//Serial.println(hex);
	digitalWrite(POT_CS, LOW);
	SPI.transfer(0);
	SPI.transfer((int)(hex));
	digitalWrite(POT_CS, HIGH);
  //resistor.setResistance(hex);
}

/*
 * Function: getChargingLimit
 * -------------------
 * This function returns the current resistor charging limit.
 */
uint8_t Charger::getChargingLimit() {
  return chargingLimit;
}
