/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Keegan Mehall | kmehall@stanford.edu

  File: Actuators.cpp
  --------------------------
  Implimentation of Actuators.h
*/

#include "Actuators.h"

//#define JANKSHITL

//extern float Slift;

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the PCB hardware.
 */
void Actuators::init() {
  pinMode(VALVE_FORWARD,    OUTPUT);
  pinMode(VALVE_REVERSE,    OUTPUT);
  pinMode(BALLAST_FORWARD,  OUTPUT);
  pinMode(BALLAST_REVERSE,  OUTPUT);
  pinMode(CUTDOWN_POWER,    OUTPUT);
  pinMode(CUTDOWN_SIGNAL,   OUTPUT);
  digitalWrite(CUTDOWN_POWER, LOW);
  digitalWrite(CUTDOWN_SIGNAL, LOW);
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
  Serial.print("Just queued");
  Serial.print(duration);
  Serial.println("ms of ballast");
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
  Serial.println("Just cleared ballast queue");
  ballastQueue = 0;
  ballastQueueFake = 0;
}

/*
 * Function: checkValve
 * -------------------
 * This function provides a non-hanging interface to check the timer queue.
 * Called every loop; updates and acts on the current state of the valve.
 */
bool Actuators::checkValve(float current) {
  // Serial.print("Called checkValve with ");
  // Serial.print(valveQueue);
  // Serial.println(" in valveQueue");
  if (valveState == CLOSED) {
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
      #ifdef JANKSHITL
        Slift -= deltaTime/1000.*0.001;
      #endif
    }
    if(valveQueue == 0) {
      valveActionStartTime = millis();
      valveState = CLOSING;
      closeValve();
    }
  }
  if ((valveState == CLOSING) && (millis() - valveActionStartTime >= valveClosingTimeout)) {
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
  // Serial.print("Called checkBallast with ");
  // Serial.print(ballastQueue);
  // Serial.print(" in ballastQueue and direction ");
  // Serial.println(ballastDirection);
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
    if (current >= stallCurrent && ((currentLast < stallCurrent) || (millis() - ballastStallTime >= BALLAST_STALL_TIMEOUT)) && (millis() - ballastDirectionTime >= BALLAST_STALL_TIMEOUT)) {
      ballastDirection = !ballastDirection;
      ballastStallTime = millis();
      numBallastOverCurrents++;
    }
    currentLast = current;
    if(ballastQueue > 0) {
      uint32_t deltaTime = (millis() - ballastCheckTime);
      ballastCheckTime = millis();
      (deltaTime >= ballastQueue) ? (ballastQueue = 0) : (ballastQueue -= deltaTime);
      #ifdef JANKSHITL
        Slift += deltaTime/1000.*0.0002;
      #endif
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
  Serial.println("starting cutdown...");
  clearValveQueue();
  clearBallastQueue();
  digitalWrite(CUTDOWN_POWER, HIGH);
  digitalWrite(CUTDOWN_SIGNAL, HIGH);
  delay(CUTDOWN_DURATION);
  digitalWrite(CUTDOWN_POWER, LOW);
  digitalWrite(CUTDOWN_SIGNAL, LOW);
  Serial.println("cutdown completed.");
}

/*
 * Function: stopValve
 * -------------------
 * This function stops the valve.
 */
void Actuators::stopValve() {
  Serial.println("--- STOPPING VALVE ---");
  analogWrite(VALVE_FORWARD, HIGH);
  analogWrite(VALVE_REVERSE, HIGH);
}

/*********************************  HELPERS  **********************************/
/*
 * Function: openValve
 * -------------------
 * This function starts opening the valve.
 */
void Actuators::openValve() {
  Serial.println("--- OPEN VALVE ---");
  analogWrite(VALVE_FORWARD, LOW);
  analogWrite(VALVE_REVERSE, valveMotorSpeedOpen);
}

/*
 * Function: closeValve
 * -------------------
 * This function starts closing the valve.
 */
void Actuators::closeValve() {
  Serial.println("--- CLOSE VALVE ---");
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
