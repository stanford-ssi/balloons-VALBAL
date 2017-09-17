/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: Actuators.cpp
  --------------------------
  Implimentation of Actuators.h
*/

#include "Actuators.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the PCB hardware.
 */
void Actuators::init() {
  pinMode(VALVE_FORWARD,   OUTPUT);
  pinMode(VALVE_REVERSE,   OUTPUT);
  pinMode(BALLAST_FORWARD, OUTPUT);
  pinMode(BALLAST_REVERSE, OUTPUT);
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateMechanicalConstants
 * -------------------
 * This function updates the mechanical constants.
 */
void Actuators::updateMechanicalConstants(uint16_t valveMotorSpeedOpenValue, uint16_t valveMotorSpeedCloseValue, uint16_t ballastMotorSpeedValue, uint32_t valveOpeningTimeoutValue, uint32_t valveClosingTimeoutValue) {
  valveMotorSpeedOpen = valveMotorSpeedOpenValue;
  valveMotorSpeedClose = valveMotorSpeedCloseValue;
  ballastMotorSpeed = ballastMotorSpeedValue;
  valveOpeningTimeout = valveOpeningTimeoutValue;
  valveClosingTimeout = valveClosingTimeoutValue;
}

/*
 * Function: queueValve
 * -------------------
 * This function increments the timer queue
 * for the mechanical valve mechanism.
 */
void Actuators::queueValve(uint32_t  duration, bool real) {
  if(real) valveQueue += duration;
  else valveQueueFake += duration;
}

/*
 * Function: queueBallast
 * -------------------
 * This function increments the timer queue
 * for the mechanical ballast mechanism.
 */
void Actuators::queueBallast(uint32_t  duration, bool real) {
  if(real) ballastQueue += duration;
  else ballastQueueFake += duration;
}

/*
 * Function: clearValveQueue
 * -------------------
 * This clears any queued valve times.
 */
void Actuators::clearValveQueue() {
  valveQueue = 0;
  valveQueueFake = 0;
}

/*
 * Function: clearBallastQueue
 * -------------------
 * This clears any queued ballast times.
 */
void Actuators::clearBallastQueue() {
  ballastQueue = 0;
  ballastQueueFake = 0;
}

/*
 * Function: checkValve
 * -------------------
 * This function provides a non-hanging interface to check the timer queue.
 * Called every loop; updates and acts on the current state of the valve.
 */
bool Actuators::checkValve(float current, uint32_t leakTimeout) {
  if (valveState == CLOSED) {
    if (((millis() - valveLeakStartTime) >= leakTimeout) && (ballastState == CLOSED)) {
      valveLeakStartTime = millis();
      valveActionStartTime = millis();
      valveState = CLOSING;
      closeValve();
    }
    if (valveQueue == 0) {
      uint32_t deltaTime = (millis() - valveCheckTime);
      valveCheckTime = millis();
      (deltaTime >= valveQueueFake) ? (valveQueueFake = 0) : (valveQueueFake -= deltaTime);
    }
    if (valveQueue > 0) {
      valveActionStartTime = millis();
      valveCheckTime = millis();
      valveState = OPENING;
      openValve();
    }
  }
  if ((valveState == OPENING) && (millis() - valveActionStartTime >= valveOpeningTimeout)) {
    valveState = OPEN;
    stopValve();
  }
  if (valveState == OPEN) {
    if(valveQueue > 0) {
      uint32_t deltaTime = (millis() - valveCheckTime);
      valveCheckTime = millis();
      (deltaTime >= valveQueue) ? (valveQueue = 0) : (valveQueue -= deltaTime);
    }
    if(valveQueue == 0) {
      valveActionStartTime = millis();
      valveState = CLOSING;
      closeValve();
    }
  }
  if ((valveState == CLOSING) && (millis() - valveActionStartTime >= valveClosingTimeout)) {
    valveLeakStartTime = millis();
    valveState = CLOSED;
    stopValve();
  }
  return valveState != CLOSED;
}

/*
 * Function: checkBallast
 * -------------------
 * This function provides a non-hanging interface to check the timer queue.
 * Called every loop; updates and acts on the current state of the ballast.
 */
bool Actuators::checkBallast(float current, uint32_t reverseTimeout, uint16_t stallCurrent) {
  if (ballastState == CLOSED) {
    if (ballastQueue == 0) {
      uint32_t deltaTime = (millis() - ballastCheckTime);
      ballastCheckTime = millis();
      (deltaTime >= ballastQueueFake) ? (ballastQueueFake = 0) : (ballastQueueFake -= deltaTime);
    }
    if (ballastQueue > 0) {
      ballastActionStartTime = millis();
      ballastCheckTime = millis();
      ballastDirectionTime = millis();
      ballastState = OPEN;
    }
  }
  if (ballastState == OPEN) {
    if (current >= stallCurrent && ((currentLast < stallCurrent) || (millis() - ballastStallTime >= BALLAST_STALL_TIMEOUT))) {
      ballastDirection = !ballastDirection;
      ballastStallTime = millis();
      numBallastOverCurrents++;
    }
    currentLast = current;
    if(ballastQueue > 0) {
      uint32_t deltaTime = (millis() - ballastCheckTime);
      ballastCheckTime = millis();
      (deltaTime >= ballastQueue) ? (ballastQueue = 0) : (ballastQueue -= deltaTime);
      if ((millis() - ballastDirectionTime) >= reverseTimeout) {
        ballastDirectionTime = millis();
        ballastDirection = !ballastDirection;
      }
      dropBallast(ballastDirection);
    }
    if(ballastQueue == 0) {
      ballastState = CLOSED;
      stopBallast();
    }
  }
  return ballastState != CLOSED;
}

/*
 * Function: getValveQueue
 * -------------------
 * This function returns the current valve queue.
 */
uint32_t Actuators::getValveQueue() {
  return valveQueue + valveQueueFake;
}

/*
 * Function: getBallastQueue
 * -------------------
 * This function returns the current ballast queue.
 */
uint32_t Actuators::getBallastQueue() {
  return ballastQueue + ballastQueueFake;
}

/*
 * Function: getNumBallastOverCurrents
 * -------------------
 * This function returns the number of times that the ballast has over currented.
 */
uint32_t Actuators::getNumBallastOverCurrents() {
  return numBallastOverCurrents;
}

/*
 * Function: clearBallastOverCurrents
 * -------------------
 * This function clears the number of times that the ballast has over currented.
 */
void Actuators::clearBallastOverCurrents() {
  numBallastOverCurrents = 0;
}

/*
 * Function: cutDown
 * -------------------
 * This function triggers the mechanical cutdown of the payload.
 */
void Actuators::cutDown() {
  clearValveQueue();
  clearBallastQueue();
  for(size_t i = 0; i < 3; i++){
    openValve();
    delay(CUTDOWN_DURATION);
    closeValve();
    delay(valveClosingTimeout);
    stopValve();
  }
}

/*
 * Function: stopValve
 * -------------------
 * This function stops the valve.
 */
void Actuators::stopValve() {
  analogWrite(VALVE_FORWARD, LOW);
  analogWrite(VALVE_REVERSE, LOW);
}

/*********************************  HELPERS  **********************************/
/*
 * Function: openValve
 * -------------------
 * This function starts opening the valve.
 */
void Actuators::openValve() {
  analogWrite(VALVE_FORWARD, LOW);
  analogWrite(VALVE_REVERSE, valveMotorSpeedOpen);
}

/*
 * Function: closeValve
 * -------------------
 * This function starts closing the valve.
 */
void Actuators::closeValve() {
  analogWrite(VALVE_FORWARD, valveMotorSpeedClose);
  analogWrite(VALVE_REVERSE, LOW);
}

/*
 * Function: stopBallast
 * -------------------
 * This function stops the ballast.
 */
void Actuators::stopBallast() {
  analogWrite(BALLAST_FORWARD, LOW);
  analogWrite(BALLAST_REVERSE, LOW);
}

/*
 * Function: dropBallast
 * -------------------
 * This function drops ballast.
 */
void Actuators::dropBallast(bool direction) {
  if (direction) {
    analogWrite(BALLAST_FORWARD, ballastMotorSpeed);
    analogWrite(BALLAST_REVERSE, LOW);
  } else {
    analogWrite(BALLAST_FORWARD, LOW);
    analogWrite(BALLAST_REVERSE, ballastMotorSpeed);
  }
}
