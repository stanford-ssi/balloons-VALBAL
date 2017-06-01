/*
  Stanford Student Space Initiative
  Balloons | VALBAL | June 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Matthew Tan | mratan@stanford.edu

  File: Hardware.cpp
  --------------------------
  Implimentation of Hardware.h
*/

#include "Hardware.h"

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the PCB hardware.
 */
void Hardware::init() {
  pinMode(REBOOT_ENABLE, OUTPUT);
  #ifdef STORAGE_MODE_FLAG
    digitalWrite(REBOOT_ENABLE, LOW);
  #endif
  #ifndef STORAGE_MODE_FLAG
    digitalWrite(REBOOT_ENABLE, HIGH);
  #endif
  pinMode(LED_PIN, OUTPUT);
  pinMode(VALVE_FORWARD, OUTPUT);
  pinMode(VALVE_REVERSE, OUTPUT);
  pinMode(BALLAST_FORWARD, OUTPUT);
  pinMode(BALLAST_REVERSE, OUTPUT);
  pinMode(HEATER_INTERNAL_STRONG, OUTPUT);
  pinMode(HEATER_INTERNAL_WEAK, OUTPUT);
  pinMode(PAYLOAD_GATE, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogWrite(HEATER_INTERNAL_STRONG, 0);
  analogWrite(HEATER_INTERNAL_WEAK, 0);
  digitalWrite(PAYLOAD_GATE, LOW);
  pid.SetMode(AUTOMATIC);
}

/*
 * Function: initResolutions
 * -------------------
 * This function is called after every pinmode is set.
 */
void Hardware::initResolutions() {
  analogReference(INTERNAL);
  analogReadResolution(12);
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: runLED
 * -------------------
 * This function turns the LED on or off.
 */
void Hardware::runLED(bool on) {
  digitalWrite(LED_PIN, on);
}

/*
 * Function: startUpHeaters
 * -------------------
 * This function starts up the heaters.
 */
bool Hardware::startUpHeaters(bool shouldStartup) {
  bool success = false;
  if (shouldStartup) {
    setHeaterMode(false);
    analogWrite(HEATER_INTERNAL_STRONG, 255);
    analogWrite(HEATER_INTERNAL_WEAK, 255);
    delay(1000);
    analogWrite(HEATER_INTERNAL_STRONG, 0);
    analogWrite(HEATER_INTERNAL_WEAK, 0);
    setHeaterMode(true);
    success = true;
  }
  return success;
}

/*
 * Function: heater
 * -------------------
 * This function runs the PID heater within the board.
 */
void Hardware::heater(double tempSetpoint, double temp, bool strong, bool weak) {
  PIDSetVar = tempSetpoint;
  PIDTempVar = temp;
  pid.Compute();
  if (PIDOutVar != 0.0) {
    if (strong)  analogWrite(HEATER_INTERNAL_STRONG, PIDOutVar / 2 + 127.5);
    if (!strong) analogWrite(HEATER_INTERNAL_STRONG, 0);
    if (weak)    analogWrite(HEATER_INTERNAL_WEAK, PIDOutVar / 2 + 127.5);
    if (!weak)   analogWrite(HEATER_INTERNAL_WEAK, 0);
  }
  else {
    analogWrite(HEATER_INTERNAL_STRONG, 0);
    analogWrite(HEATER_INTERNAL_WEAK, 0);
  }
}

/*
 * Function: turnOffHeaters
 * -------------------
 * This function shuts down the heaters.
 */
void Hardware::turnOffHeaters() {
  analogWrite(HEATER_INTERNAL_STRONG, 0);
  analogWrite(HEATER_INTERNAL_WEAK, 0);
}

/*
 * Function: setHeaterMode
 * -------------------
 * This function sets the heater mode that
 * persists through system restarts.
 */
void Hardware::setHeaterMode(bool on) {
  EEPROM.write(EEPROM_HEATER, on);
}

/*
 * Function: updateMechanicalConstants
 * -------------------
 * This function updates the mechanical constants.
 */
void Hardware::updateMechanicalConstants(uint16_t valveMotorSpeedValue, uint16_t ballastMotorSpeedValue, uint32_t valveOpeningTimeoutValue, uint32_t valveClosingTimeoutValue) {
  valveMotorSpeed = valveMotorSpeedValue;
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
void Hardware::queueValve(uint32_t  duration, bool real) {
  if(real) valveQueue += duration;
  else valveQueueFake += duration;
}

/*
 * Function: queueBallast
 * -------------------
 * This function increments the timer queue
 * for the mechanical ballast mechanism.
 */
void Hardware::queueBallast(uint32_t  duration, bool real) {
  if(real) ballastQueue += duration;
  else ballastQueueFake += duration;
}

/*
 * Function: clearValveQueue
 * -------------------
 * This clears any queued valve times.
 */
void Hardware::clearValveQueue() {
  valveQueue = 0;
  valveQueueFake = 0;
}

/*
 * Function: clearBallastQueue
 * -------------------
 * This clears any queued ballast times.
 */
void Hardware::clearBallastQueue() {
  ballastQueue = 0;
  ballastQueueFake = 0;
}

/*
 * Function: checkValve
 * -------------------
 * This function provides a non-hanging interface to check the timer queue.
 * Called every loop; updates and acts on the current state of the valve.
 */
bool Hardware::checkValve(float current, uint32_t leakTimeout) {
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
bool Hardware::checkBallast(float current, uint32_t reverseTimeout, uint16_t stallCurrent) {
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
uint32_t Hardware::getValveQueue() {
  return valveQueue + valveQueueFake;
}

/*
 * Function: getBallastQueue
 * -------------------
 * This function returns the current ballast queue.
 */
uint32_t Hardware::getBallastQueue() {
  return ballastQueue + ballastQueueFake;
}

/*
 * Function: getNumBallastOverCurrents
 * -------------------
 * This function returns the number of times that the ballast has over currented.
 */
uint32_t Hardware::getNumBallastOverCurrents() {
  return numBallastOverCurrents;
}

/*
 * Function: clearBallastOverCurrents
 * -------------------
 * This function clears the number of times that the ballast has over currented.
 */
void Hardware::clearBallastOverCurrents() {
  numBallastOverCurrents = 0;
}

/*
 * Function: cutDown
 * -------------------
 * This function triggers the mechanical cutdown of the payload.
 */
void Hardware::cutDown() {
  turnOffHeaters();
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
void Hardware::stopValve() {
  analogWrite(VALVE_FORWARD, LOW);
  analogWrite(VALVE_REVERSE, LOW);
}


/*
 * Function: EEPROMReadlong
 * -------------------
 * This function reads an int32_t from the given address.
 */
int32_t Hardware::EEPROMReadlong(uint8_t address) {
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
void Hardware::EEPROMWritelong(uint8_t address, int32_t value) {
  uint8_t four = (value & 0xFF);
  uint8_t three = ((value >> 8) & 0xFF);
  uint8_t two = ((value >> 16) & 0xFF);
  uint8_t one = ((value >> 24) & 0xFF);
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

/*********************************  HELPERS  **********************************/
/*
 * Function: openValve
 * -------------------
 * This function starts opening the valve.
 */
void Hardware::openValve() {
  analogWrite(VALVE_FORWARD, LOW);
  analogWrite(VALVE_REVERSE, valveMotorSpeed);
}

/*
 * Function: closeValve
 * -------------------
 * This function starts closing the valve.
 */
void Hardware::closeValve() {
  analogWrite(VALVE_FORWARD, valveMotorSpeed);
  analogWrite(VALVE_REVERSE, LOW);
}

/*
 * Function: stopBallast
 * -------------------
 * This function stops the ballast.
 */
void Hardware::stopBallast() {
  analogWrite(BALLAST_FORWARD, LOW);
  analogWrite(BALLAST_REVERSE, LOW);
}

/*
 * Function: dropBallast
 * -------------------
 * This function drops ballast.
 */
void Hardware::dropBallast(bool direction) {
  if (direction) {
    analogWrite(BALLAST_FORWARD, ballastMotorSpeed);
    analogWrite(BALLAST_REVERSE, LOW);
  } else {
    analogWrite(BALLAST_FORWARD, LOW);
    analogWrite(BALLAST_REVERSE, ballastMotorSpeed);
  }
}
