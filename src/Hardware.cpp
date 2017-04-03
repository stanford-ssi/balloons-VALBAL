/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
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
  digitalWrite(REBOOT_ENABLE, HIGH);
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
  analogReference(INTERNAL);
  analogReadResolution(12);
  pid.SetMode(AUTOMATIC);
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
 * Function: faultLED
 * -------------------
 * This function alerts the user if there has been a fatal error.
 */
void Hardware::faultLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(LOOP_INTERVAL);
  digitalWrite(LED_PIN, LOW);
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
  EEPROM.write(EEPROMAddress, on);
}

/*
 * Function: queueValve
 * -------------------
 * This function increments the timer queue
 * for the mechanical valve mechanism.
 */
void Hardware::queueValve(int duration) {
  valveQueue += duration;
}

/*
 * Function: queueBallast
 * -------------------
 * This function increments the timer queue
 * for the mechanical ballast mechanism.
 */
void Hardware::queueBallast(int duration) {
  ballastQueue += duration;
}

/*
 * Function: clearValveQueue
 * -------------------
 * This clears any queued valve times.
 */
void Hardware::clearValveQueue() {
  valveQueue = 0;
}

/*
 * Function: clearBallastQueue
 * -------------------
 * This clears any queued ballast times.
 */
void Hardware::clearBallastQueue() {
  ballastQueue = 0;
}

/*
 * Function: checkValve
 * -------------------
 * This function provides a non-hanging interface to check the timer queue.
 * Called every loop; updates and acts on the current state of the valve.
 */
bool Hardware::checkValve() {
  if (valveState == OPENING) {
    if (millis() - valveActionStartTime >= VALVE_OPENING_TIMEOUT) { // exceeded opening time
      valveState = OPEN;
    } else {
      openValve();
    }
  } else if (valveState == OPEN) {
    if (millis() - valveActionStartTime >= valveQueue) { // exceeded queued time
      valveQueue = 0;
      valveActionStartTime = millis();
      valveState = CLOSING;
    } else {
      stopValve();
    }
  } else if (valveState == CLOSING) {
    if (millis() - valveActionStartTime >= VALVE_CLOSING_TIMEOUT) { // exceeded closing time
      valveState = CLOSED;
    } else {
      closeValve();
    }
  } else if (valveState == CLOSED) {
    if (valveQueue > 0) { // we've got mail!
      valveActionStartTime = millis();
      valveState = OPENING;
    } else {
      stopValve();
    }
  }
  if((valveState == CLOSED) && (millis() - valveLeakStartTime) >= VALVE_LEAK_TIMEOUT) {
    valveLeakStartTime = millis();
    closeValve();
  }
  return valveState != CLOSED;
}

/*
 * Function: checkBallast
 * -------------------
 * This function provides a non-hanging interface to check the timer queue.
 * Called every loop; updates and acts on the current state of the ballast.
 */
bool Hardware::checkBallast() {
  if (ballastState == OPEN) {
    if (millis() - ballastActionStartTime >= ballastQueue) { // exceeded queued time
      ballastQueue = 0;
      ballastState = CLOSED;
    } else {
      // every BALLAST_REVERSE_TIMEOUT milliseconds, switch directions
      ballastDirection = ((millis() - ballastActionStartTime) / BALLAST_REVERSE_TIMEOUT) % 2;
      dropBallast(ballastDirection);
    }
  } else if (ballastState == CLOSED) {
    if (ballastQueue > 0) {
      ballastActionStartTime = millis();
      ballastState = OPEN;
    } else {
      stopBallast();
    }
  }
  return ballastState != CLOSED;
}

/*
 * Function: isValveRunning
 * -------------------
 * This function checks if the valve is running.
 */
bool Hardware::isValveRunning() {
  return valveState == OPENING || valveState == CLOSING;
}

/*
 * Function: isBallastRunning
 * -------------------
 * This function checks if the ballast is running.
 */
bool Hardware::isBallastRunning() {
  return ballastState == OPEN;
}

/*
 * Function: getValveQueue
 * -------------------
 * This function returns the current valve queue.
 */
uint32_t Hardware::getValveQueue() {
  return valveQueue;
}

/*
 * Function: getBallastQueue
 * -------------------
 * This function returns the current ballast queue.
 */
uint32_t Hardware::getBallastQueue() {
  return ballastQueue;
}

/*
 * Function: cutDown
 * -------------------
 * This function triggers the mechanical cutdown of the payload.
 */
void Hardware::cutDown(bool on) {
  turnOffHeaters();
  clearValveQueue();
  clearBallastQueue();
  if(on) {
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, VALVE_MOTOR_SPEED);
  }
  else {
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, LOW);
  }
}

/*
 * Function: writeToEEPROM
 * -------------------
 * This helper function writes an integer digit-by-digit
 * to EEPROM between the specified bytes.
 */
void Hardware::writeToEEPROM(uint8_t startByte, uint8_t endByte, int num) {
  // write from left to right (endByte to startByte) b/c writing from one's digit
	for (int pos = endByte; pos >= startByte; pos--) {
		int digit = num % 10;
		num /= 10;
		EEPROM.write(pos, digit);
	}
}

/*
 * Function: writeToEEPROM
 * -------------------
 * This helper function reads an integer digit-by-digit
 * from EEPROM between the specified bytes, and then "clears"
 * the data by writing the CLEAR_NUM sentinel.
 */
int Hardware::readFromEEPROMAndClear(uint8_t startByte, uint8_t endByte) {
  int num = 0;
  // build up number
  for (int i = startByte; i <= endByte; i++) {
    if (EEPROM.read(0) == EEPROM_CLEAR_NUM) break;
    int digit = EEPROM.read(i);
    num *= 10;
    num += digit;
  }
  // clear EEPROM data
  writeToEEPROM(startByte, endByte, EEPROM_CLEAR_NUM);
  return num;
}

/*********************************  HELPERS  **********************************/
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
 * Function: openValve
 * -------------------
 * This function starts opening the valve.
 */
void Hardware::openValve() {
  analogWrite(VALVE_FORWARD, LOW);
  analogWrite(VALVE_REVERSE, VALVE_MOTOR_SPEED);
}

/*
 * Function: closeValve
 * -------------------
 * This function starts closing the valve.
 */
void Hardware::closeValve() {
  analogWrite(VALVE_FORWARD, VALVE_MOTOR_SPEED);
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
  if (ballastDirection) {
    analogWrite(BALLAST_FORWARD, BALLAST_MOTOR_SPEED);
    analogWrite(BALLAST_REVERSE, LOW);
  } else {
    analogWrite(BALLAST_FORWARD, LOW);
    analogWrite(BALLAST_REVERSE, BALLAST_MOTOR_SPEED);
  }
}
