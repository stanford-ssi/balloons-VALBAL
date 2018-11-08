/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Aria Tedjarati | atedjarati@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  John Dean | deanjl@stanford.edu
  Ben Newman | blnewman@stanford.edu
  Keegan Mehall | kmehall@stanford.edu
  Jonathan Zwiebel | jzwiebel@stanford.edu

  File: Avionics.cpp
  --------------------------
  Implementation of Avionics.h
*/

#include "Avionics.h"

//#define Serial Serial2

int numExecutions = 0;

void rpmCounter() {
  numExecutions++;
}

IntervalTimer sixtyScoreRevolutionsPerMinute;

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the avionics flight controller.
 */
void Avionics::init() {
  numExecutions = 0;
  #ifdef JANKSHITL
  //stepsim.setSS(101000);
  //tempsim.setSS(0);
  #endif
  Serial.begin(CONSOLE_BAUD);
  delay(2000);
  PCB.init();
  actuator.init();
  delay(500);
  Serial.println("setting payload high");

/*
  // here be heaters
  pinMode(36, OUTPUT);
  pinMode(OP_PIN, INPUT);
  pinMode(VR_PIN, INPUT);

  pinMode(57, OUTPUT);
  digitalWrite(57, LOW);*/

  if(!setupSDCard())                          alert("unable to initialize SD Card", true);
  if(!readHistory())                          alert("unable to initialize EEPROM", true);
  if(!sensors.init())                         alert("unable to initialize Sensors", true);

	//return;
  //delay(2000);
  Serial.println("Serial has been init");
  //if(!currentSensor.init(CURRENT_MONITOR_CS)) alert("unable to initialize Current Sensor", true);
// #ifdef HITL_ENABLED_FLAG
//   if(!HITL.init())                            alert("unable to initialize Simulations", true);
// #endif
  if(!computer.init())                        alert("unable to initialize Flight Controller", true) ;
  //gpsModule.GPS_MODE = 1;
  if(!gpsModule.init(!false))   alert("unable to initialize GPS", true);
  if(!superCap.init())                        alert("unable to initialize superCap", true);
  if(!setup5VLine())                          alert("unable to initialize 5V line", true);
  // pinMode(49, OUTPUT);
  // digitalWrite(49, HIGH);
  // pinMode(56, OUTPUT);
  // digitalWrite(56, HIGH);
#ifndef RB_DISABLED_FLAG
/*pinMode(RB_GATE, OUTPUT);
digitalWrite(RB_GATE, LOW);
delay(2500);
Serial.println("setting RB gate high");
digitalWrite(RB_GATE, HIGH);
delay(1000);*/
//Serial1.begin(19200);
	/*while(millis() < 60000) {
		Serial1.write(0xaa);
	}*/
  if(!RBModule.init(false))     alert("unable to initialize RockBlock", true);
#endif
  //if(!radio.init(data.POWER_STATE_RADIO)) alert("unable to initialize Payload", true);
	delay(1000);

/*
  pinMode(RB_GATE, OUTPUT);
  digitalWrite(RB_GATE, LOW);
	*/
  data.TIME = millis();
  data.SETUP_STATE = false;


  #if defined(SERIALSHITL) | defined(SERIALMONITOR)
  //holds until connection with PC is established
  Serial.write(FSTART);
  while(true){
    delay(50);
    if(Serial.available()){
      if(Serial.read() == FSTART){
        break;
      }
    }
  }
  #endif

	pinMode(25, OUTPUT);
	/*pixels.begin();
  pixels.setBrightness(128);
  pixels.show();*/
  sixtyScoreRevolutionsPerMinute.priority(16);
  sixtyScoreRevolutionsPerMinute.begin(rpmCounter, 50000);
}

/*
 * Function: test
 * -------------------
 * This function tests the hardware.
 */
void Avionics::test() {
  alert("Initializing test...", true);

  //actuator.queueBallast(40000, true);
  //actuator.queueValve(10000, true);
  //actuator.queueValve(30000, true)
  /*data.SHOULD_CUTDOWN = true;
  actuator.cutDown();
  data.SHOULD_CUTDOWN = false;*/
}

void Avionics::runHeaters() {
  if (!data.POWER_STATE_LED) {
    //analogWrite(36, 0);
    //data.RB_HEAT_DUTY = 0;
    return;
  }
  heater.updateConstants(data.HEATER_CONSTANTS);

  Heater::Input heaterInput;
  heaterInput.temp_int = data.TEMP_INT;
  heaterInput.voltage_supercap_average = data.VOLTAGE_SUPERCAP_AVG;
  heaterInput.rb_interval = data.RB_INTERVAL;
  heaterInput.rb_last = data.RB_LAST;
  data.RB_HEAT_DUTY = heater.update(heaterInput);
}


/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateState
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateState() {
  #ifndef HITL_ENABLED_FLAG
    if(!readData())     alert("unable to read Data", true);
  #endif
// #ifdef HITL_ENABLED_FLAG
//   if(!simulateData()) alert("unable to simulate Data", true);
// #endif
  if(!processData())  alert("unable to process Data", true);
/*
	  if (data.TIME > 30000 && data.TIME < 30050) {
	    actuator.queueBallast(10000, true);
	  }
		  if (data.TIME > 40000 && data.TIME < 40050) {
		    actuator.queueValve(10000, true);
		  }*/
  //currentSensor.read_voltage(DIFF_12_13);
  //Serial.print("avg voltage: ");
  //uint32_t t0 = micros();
  //Serial.println(currentSensor.average_voltage_readings(DIFF_12_13, CURRENT_NUM_SAMPLES), 6);
  //uint32_t dt = micros() - t0;
  //Serial.println(dt);
  //delay(LOOP_INTERVAL);
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently calculates the current state.
 */
void Avionics::evaluateState() {
  if(!calcVitals())     alert("unable to calculate vitals", true);
  if(!calcDebug())      alert("unable to calculate debug", true);
  if(!calcIncentives()) alert("unable to calculate incentives", true);
}

/*
 * Function: actuateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::actuateState() {
  if(!runCharger()) alert("unable to run charger", true);
  if(!runValve())   alert("unable to run valve", true);
  if(!runBallast()) alert("unable to run ballast", true);
  if(!runCutdown()) alert("unable to run cutdown", true);
  if(!runLED())     alert("unable to run LED", true);
  //runHeaters();
  rumAndCoke();
  timedCutdown();
  runDeadMansSwitch();
  //if(!runRadio()) alert("Unable to run payload", true);

}

/*
 * Function: logState
 * -------------------
 * This function logs the current data frame.
 */
void Avionics::logState() {
  uint32_t t0 = millis();
  //Serial.println("begin");
	data.NATURAL_NUMBERS++;
  if(!log.log(&data, sizeof(data))) alert("unable to log Data", true);
  //Serial.println("end");
  data.LOG_TIME = millis() - t0;
  if(!debugState())   alert("unable to debug state", true);
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
 void Avionics::sendComms() {
 #ifndef RB_DISABLED_FLAG
   if(!data.VALVE_STATE && !data.POWER_STATE_RB && ((millis() - data.RB_LAST) > RB_RESTART_INTERVAL)) {
     Serial.println("RB emergency auto-restart");
     RBModule.restart();
     data.POWER_STATE_RB = true;
   }
   //Serial.println((millis()-data.RB_LAST)/RB_DEBUG_INTERVAL);
   if(data.DEBUG_STATE && ((millis() - data.RB_LAST) < RB_DEBUG_INTERVAL)) return;
   if(!data.DEBUG_STATE && ((millis() - data.RB_LAST) < data.RB_INTERVAL)) return;
   if (!data.POWER_STATE_RB) {
     Serial.println("normally would've comm'd here, but we're too rekt to do that rn.");
     return;
   }
   if(compressData() < 0) alert("unable to compress Data", true);
   if(!sendSATCOMS())  {
     alert("unable to communicate over RB", true);
     // cooldown for 30 seconds
     uint32_t interval = data.RB_INTERVAL;
     if (data.DEBUG_STATE) interval = RB_DEBUG_INTERVAL;
     if (millis() > interval) {
       data.RB_LAST = millis() - interval + data.RB_COOLDOWN;
     } else {
       data.RB_LAST = millis();
     }
   }
   else {
      data.RB_LAST = millis();
      data.TIME_LAST_COMM = millis();
   }
 #endif
 }

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  uint32_t loopTime = millis() - data.TIME;
  if (loopTime < LOOP_INTERVAL) gpsModule.smartDelay(LOOP_INTERVAL - loopTime);
  data.LOOP_NUMBER++;
}

/*
 * Function: finishedSetup
 * -------------------
 * This function returns true if the avionics has completed setup.
 */
bool Avionics::finishedSetup() {
  return !data.SETUP_STATE;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: setupSDCard
 * -------------------
 * This function sets up the SD card for logging.
 */
bool Avionics::setupSDCard() {
  Serial.println("calling initialize");
  bool ret = log.initialize();
  #ifdef WIPE_SD_CARD_YOLO
    ret = log.wipe();
  #endif
    Serial.println("done initialize");

  return ret;
}

/*
 * Function: setup5VLine
 * -------------------
 * This function charges the superCap to 5 Volts.
 */
bool Avionics::setup5VLine() {
  while(sensors.getVoltageSuperCap() <= SUPER_CAP_VOLTAGE_NOMINAL) {
    delay(LOOP_INTERVAL);
    Serial.print("SuperCap is currently at: ");
    Serial.print(sensors.getVoltageSuperCap());
    Serial.println(" Volts.");
  }
  superCap.enable5VBoost();
  return true;
}

/*
 * Function: clearVariables
 * -------------------
 * This function clears the interval based variables at the end of
 * a successfull transmission.
 */
void Avionics::clearVariables() {
  filter.clear();
  actuator.clearBallastOverCurrents();
  for (int i=0; i<4; i++) data.BMP_REJECTIONS[i] = 0;
  data.VALVE_NUM_ACTIONS = 0;
  data.BALLAST_NUM_ACTIONS = 0;
  data.VALVE_NUM_ATTEMPTS = 0;
  data.BALLAST_NUM_ATTEMPTS = 0;
  data.LOOP_TIME_MAX = 0;
  data.RB_RESTARTS = 0;
  data.VOLTAGE_SUPERCAP_MIN = 314;
  int len = sizeof(data.ACTION_TIME_TOTALS)/sizeof(data.ACTION_TIME_TOTALS[0]);
  for(int i=0; i < len; i++){
    data.ACTION_TIME_TOTALS[i] = 0;
  }
}

int Avionics::numExecReset() {
  noInterrupts();
    int numExecNow = numExecutions;
    numExecutions = 0;
  interrupts();
  return numExecNow;
}
