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
  pid.SetMode(AUTOMATIC);
}

/********************************  FUNCTIONS  *********************************/

/*********************************  HELPERS  **********************************/
