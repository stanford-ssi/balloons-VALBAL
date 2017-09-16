/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
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
bool Charger::init() {
  pinMode(SUPER_CAP_ENABLE, OUTPUT);
  pinMode(FIVE_VOLT_ENABLE, OUTPUT);
  digitalWrite(SUPER_CAP_ENABLE, LOW);
  digitalWrite(FIVE_VOLT_ENABLE, LOW);
  if (resistor.init()){
    digitalWrite(SUPER_CAP_ENABLE, HIGH);
    return true;
  }
  return false;
}

/********************************  FUNCTIONS  *********************************/
void Charger::enable5VBoost() {
  digitalWrite(FIVE_VOLT_ENABLE, HIGH);
}
void Charger::disable5VBoost() {
  digitalWrite(FIVE_VOLT_ENABLE, LOW);
}

/*
 * Function: runCharger
 * -------------------
 * This function calcualtes and updates the desired charging output.
 */
void Charger::runCharger(float temp) {
  float resistanceCur = resistor.getCurrentResistance();
  float currentCurr = 10000 / resistanceCur;
}

/*********************************  HELPERS  **********************************/
