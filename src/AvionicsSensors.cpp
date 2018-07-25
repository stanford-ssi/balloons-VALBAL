#include "Avionics.h"

/*
 * Function: readHistory
 * -------------------
 * This function updates the data frame with values from EEPROM
 * if avionics is restarted mid flight.
 */
bool Avionics::readHistory() {
#ifdef RESET_EEPROM_FLAG
  for(size_t i = 0; i < 1023; i++)   EEPROM.write(i, 0x0);
    EEPROM.write(EEPROM_ROCKBLOCK, true);
    EEPROM.write(EEPROM_GPS, true);
    EEPROM.write(EEPROM_PAYLOAD, true);
    PCB.EEPROMWritelong(EEPROM_VALVE_ALT_LAST, data.VALVE_ALT_LAST);
    PCB.EEPROMWritelong(EEPROM_BALLAST_ALT_LAST, data.BALLAST_ALT_LAST);
#endif
#ifndef RESET_EEPROM_FLAG
  if(!EEPROM.read(EEPROM_ROCKBLOCK)) data.POWER_STATE_RB = false;
  if(!EEPROM.read(EEPROM_GPS)) data.POWER_STATE_GPS = false;
  if(!EEPROM.read(EEPROM_PAYLOAD)) data.POWER_STATE_PAYLOAD = false;
  double valveAltLast = PCB.EEPROMReadlong(EEPROM_VALVE_ALT_LAST);
  if (valveAltLast != 0) data.VALVE_ALT_LAST = valveAltLast;
  double ballastAltLast = PCB.EEPROMReadlong(EEPROM_BALLAST_ALT_LAST);
  if (ballastAltLast != 0) data.BALLAST_ALT_LAST = ballastAltLast;
#endif
  return true;
}

/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
bool Avionics::readData() {
  data.LOOP_TIME                  = millis() - data.TIME;
  data.TIME                       = millis();
  data.VOLTAGE_PRIMARY            = sensors.getVoltagePrimary();
  data.VOLTAGE_SUPERCAP           = sensors.getVoltageSuperCap();
  data.CURRENT_TOTAL              = -2*currentSensor.average_voltage_readings(DIFF_10_11, CURRENT_NUM_SAMPLES);
  data.JOULES_TOTAL               += data.LOOP_TIME/1000.*data.VOLTAGE_PRIMARY*data.CURRENT_TOTAL/1000.;
  data.CURRENT_RB                 = -currentSensor.average_voltage_readings(DIFF_14_15, CURRENT_NUM_SAMPLES);
  data.MAX_CURRENT_CHARGING_LIMIT = superCap.getChargingLimit();
  float motcur = currentSensor.average_voltage_readings(DIFF_8_9, CURRENT_NUM_SAMPLES);
  data.CURRENT_MOTOR_VALVE        = (data.VALVE_STATE ? motcur : 0);
  data.CURRENT_MOTOR_BALLAST      = (data.BALLAST_STATE ? motcur : 0);
  data.CURRENT_MOTORS             = motcur;
  data.CURRENT_PAYLOAD            = -currentSensor.average_voltage_readings(DIFF_12_13, CURRENT_NUM_SAMPLES);
  data.TEMP_EXT                   = sensors.getDerivedTemp(EXT_TEMP_SENSOR);
  #ifdef SERIALSHITL
  shitlUpdate();
  #else
  data.RAW_TEMP_1                 = sensors.getRawTemp(1);
  data.RAW_TEMP_2                 = sensors.getRawTemp(2);
  data.RAW_TEMP_3                 = sensors.getRawTemp(3);
  data.RAW_TEMP_4                 = sensors.getRawTemp(4);
  data.RAW_PRESSURE_1             = sensors.getRawPressure(1);
  data.RAW_PRESSURE_2             = sensors.getRawPressure(2);
  data.RAW_PRESSURE_3             = sensors.getRawPressure(3);
  data.RAW_PRESSURE_4             = sensors.getRawPressure(4);
  #endif
  #ifdef JANKSHITL
  float v;
  if (Slift >= 0) {
    v = 5*sqrt(Slift);
  } else {
    v = -5*sqrt(-Slift);
  }
  Salt += v*data.LOOP_TIME/1000.;
  float p = 3.86651e-20 * pow(44330-Salt,5.255);
  data.RAW_PRESSURE_1             = p;
  data.RAW_PRESSURE_2             = p;
  data.RAW_PRESSURE_3             = p;
  data.RAW_PRESSURE_4             = p;
  #endif
  #ifdef SERIALMONITOR
  serialMonitorUpdate();
  #endif
  data.RAW_TEMP_1 = (isnan(data.RAW_TEMP_1) ? 0 : data.RAW_TEMP_1);
  data.RAW_TEMP_2 = (isnan(data.RAW_TEMP_2) ? 0 : data.RAW_TEMP_2);
  data.RAW_TEMP_3 = (isnan(data.RAW_TEMP_3) ? 0 : data.RAW_TEMP_3);
  data.RAW_TEMP_4 = (isnan(data.RAW_TEMP_4) ? 0 : data.RAW_TEMP_4);
  data.RAW_PRESSURE_1 = (isnan(data.RAW_PRESSURE_1) ? 0 : data.RAW_PRESSURE_1);
  data.RAW_PRESSURE_2 = (isnan(data.RAW_PRESSURE_2) ? 0 : data.RAW_PRESSURE_2);
  data.RAW_PRESSURE_3 = (isnan(data.RAW_PRESSURE_3) ? 0 : data.RAW_PRESSURE_3);
  data.RAW_PRESSURE_4 = (isnan(data.RAW_PRESSURE_4) ? 0 : data.RAW_PRESSURE_4);
  if (data.POWER_STATE_GPS && ((millis() - data.GPS_LAST) >= data.GPS_INTERVAL) && (!data.VALVE_STATE)) readGPS();
  return true;
}

/*
 * Function: readGPS
 * -------------------
 * This function reads data from the GPS module.
 */
bool Avionics::readGPS() {
  Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();
  Serial.println("READ GPS CALLED");
  gpsModule.smartDelay(GPS_LOCK_TIMEOUT);
  bool new_values = false;
  float new_lat         = gpsModule.getLatitude();
  float new_long        = gpsModule.getLongitude();
  if(new_lat != data.LAT_GPS || new_long != data.LONG_GPS) {
      new_values = true;
  }
  data.LAT_GPS = new_lat;
  data.LONG_GPS = new_long;
  data.ALTITUDE_GPS     = gpsModule.getAltitude();
  data.HEADING_GPS      = gpsModule.getCourse();
  data.SPEED_GPS        = gpsModule.getSpeed();
  data.NUM_SATS_GPS     = gpsModule.getSats();

  if(new_values) {
    Serial.println("Fresh GPS values coming in");
    data.GPS_TIME.year    = gpsModule.getYear();
    data.GPS_TIME.month   = gpsModule.getMonth();
    data.GPS_TIME.day     = gpsModule.getDay();
    data.GPS_TIME.hour    = gpsModule.getHour();
    data.GPS_TIME.minute  = gpsModule.getMinute();
    data.GPS_TIME.second  = gpsModule.getSecond();
    data.GPS_LAST_NEW     = millis();
    if(data.GPS_MANUAL_MODE_OVERRIDE) {
      data.GPS_MANUAL_MODE = false;
    }
  }
  data.GPS_LAST         = millis();
  Serial.println();Serial.println();Serial.println();Serial.println();
  return true;
}

float Salt = 13000;
float Slift = 0.1;

#ifdef HITL_ENABLED_FLAG
bool Avionics::simulateData() {
  DataFrame simulation = HITL.readData();
  data.LOOP_TIME                 = millis() - data.TIME;
  data.TIME                      = millis();
  data.RAW_PRESSURE_1            = simulation.RAW_PRESSURE_1;
  data.RAW_PRESSURE_2            = simulation.RAW_PRESSURE_2;
  data.RAW_PRESSURE_3            = simulation.RAW_PRESSURE_3;
  data.RAW_PRESSURE_4            = simulation.RAW_PRESSURE_4;
  data.BMP_1_ENABLE              = simulation.BMP_1_ENABLE;
  data.BMP_2_ENABLE              = simulation.BMP_2_ENABLE;
  data.BMP_3_ENABLE              = simulation.BMP_3_ENABLE;
  data.BMP_4_ENABLE              = simulation.BMP_4_ENABLE;
  data.PRESS_BASELINE            = simulation.PRESS_BASELINE;
  data.BALLAST_ARM_ALT           = simulation.BALLAST_ARM_ALT;
  data.VALVE_SETPOINT            = simulation.VALVE_SETPOINT;
  data.VALVE_VENT_DURATION       = simulation.VALVE_VENT_DURATION;
  data.VALVE_VELOCITY_CONSTANT   = simulation.VALVE_VELOCITY_CONSTANT;
  data.BALLAST_SETPOINT          = simulation.BALLAST_SETPOINT;
  data.BALLAST_DROP_DURATION     = simulation.BALLAST_DROP_DURATION;
  data.BALLAST_VELOCITY_CONSTANT = simulation.BALLAST_VELOCITY_CONSTANT;
  return true;
}
#endif
