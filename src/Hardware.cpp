/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Matthew Tan | mratan@stanford.edu

  File: Hardware.cpp
  --------------------------
  Implimentation of Hardware.h
*/

#include "Hardware.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the PCB hardware.
*/
void Hardware::init() {
  pinMode(VALVE_REVERSE, OUTPUT);
  pinMode(VALVE_FORWARD, OUTPUT);
  pinMode(BALLAST_REVERSE, OUTPUT);
  pinMode(BALLAST_FORWARD, OUTPUT);
  pinMode(HEATER_INTERNAL_STRONG, OUTPUT);
  pinMode(HEATER_INTERNAL_WEAK, OUTPUT);
  pinMode(PAYLOAD_GATE, OUTPUT);
  digitalWrite(PAYLOAD_GATE, LOW);
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: faultLED
 * -------------------
 * This function alerts the user if there has been a fatal error.
 */
void Hardware::faultLED() {
  digitalWrite(FAULT_PIN, HIGH);
  delay(LOOP_RATE);
  digitalWrite(FAULT_PIN, LOW);
}

/*
  function: heater
  ---------------------------------
  This function runs the PID heater within the board.
*/
void Hardware::heater(double temp) {
  PIDTempVar = temp;
  pid.Compute();
  if (PIDOutVar != 0.0) analogWrite(HEATER_INTERNAL_STRONG, PIDOutVar / 2 + (ANALOG_MAX / 2));
  else analogWrite(HEATER_INTERNAL_STRONG, 0);
}

/*
  function: valve
  ---------------------------------
  This function queues the mechanical valve mechanism.
*/
void Hardware::queueValve(bool force) {
  //do not hang
  //add to priority queue
  //look at state
}

/*
  function: ballast
  ---------------------------------
  This function queues the mechanical ballast mechanism.
*/
void Hardware::queueBallast(bool force) {
  //do not hang
  //add to priority queue
  //look at state
}

/*
  function: checkValve
  ---------------------------------
  This function provides a non-hanging interface to check the hardware timers.
*/
bool Hardware::checkValve() {
  return isValveOn;
}

/*
  function: checkBallast
  ---------------------------------
  This function provides a non-hanging interface to check the hardware timers.
*/
bool Hardware::checkBallast() {
  //change direction after time
  return isBallastOn;
}

/*
  function: cutDown
  ---------------------------------
  This function triggers the mechanical cutdown of the payload.
*/
void Hardware::cutDown(bool on) {
  //full motor engagement
  //remeber to turn off so we do not waste power on stall torque
  //clear valve and ballast quues cuz that doenst matter anymore
  // if(on) // engage cutdown
  // else //disengage cutdown
}
