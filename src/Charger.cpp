/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
  Davy Ragland | dragland@stanford.edu

  File: Charger.cpp
  --------------------------
  Implimentation of Charger.h
*/

#include "Charger.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the PCB hardware.
 */
void Charger::init() {
  pinMode(SUPER_CAP_ENABLE, OUTPUT);
  pinMode(FIVE_VOLT_ENABLE, OUTPUT);
  digitalWrite(SUPER_CAP_ENABLE, LOW);
  digitalWrite(FIVE_VOLT_ENABLE, LOW);
  if (resistor.init()){
    digitalWrite(SUPER_CAP_ENABLE, HIGH);
  }
  pid.SetMode(AUTOMATIC);
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: runCharger
 * -------------------
 * This function calcualtes and updates the desired charging output.
 */
void Charger::runCharger() {
  float resistance = resistor.getCurrentResistance();
  float current = 10000 / resistance;
}

/*********************************  HELPERS  **********************************/
