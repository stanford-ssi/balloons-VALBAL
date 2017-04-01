/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
  Davy Ragland | dragland@stanford.edu

  File: Config.h
  --------------------------
  Global constants specific to each launch.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   REBOOT_ENABLE                 =                       16;
static const uint8_t   SD_CS                         =                       23;
static const uint8_t   LED_PIN                       =                       33;
static const uint8_t   BMP_CS_ONE                    =                       32;
static const uint8_t   BMP_CS_TWO                    =                       25;
static const uint8_t   BMP_CS_THREE                  =                       15;
static const uint8_t   BMP_CS_FOUR                   =                       22;
static const uint8_t   VALVE_FORWARD                 =                        6;
static const uint8_t   VALVE_REVERSE                 =                       20;
static const uint8_t   BALLAST_FORWARD               =                       21;
static const uint8_t   BALLAST_REVERSE               =                        5;
static const uint8_t   HEATER_INTERNAL_STRONG        =                        4;
static const uint8_t   HEATER_INTERNAL_WEAK          =                        3;
static const uint8_t   GPS_GATE                      =                       17;
static const uint8_t   RB_GATE                       =                       28;
static const uint8_t   RB_SLEEP                      =                       14;
static const uint8_t   PAYLOAD_GATE                  =                       24;
static const uint8_t   BATT_VOLTAGE                  =                      A14;
static const uint8_t   BATT_CURRENT                  =                      A10;
static const uint8_t   EXTERNAL_CURRENT              =                      A11;
static const uint8_t   GPS_CURRENT                   =                        1;
static const uint8_t   RB_CURRENT                    =                        2;
static const uint8_t   Motors_CURRENT                =                        3;
static const uint8_t   Payload_CURRENT               =                        4;
static const uint8_t   NECK_TEMP_SENSOR              =                       A2;

/****************************  EDITABLE CONSTANTS  ****************************/
static const char      MISSION_NUMBER[]              =                 "SSI-51";
static const char      CSV_DATA_HEADER[]             =    "TIME,LAT_GPS,LONG_GPS,ALTITUDE,ALTITUDE_GPS,ASCENT_RATE,VALVE_INCENTIVE,BALLAST_INCENTIVE,VALVE_STATE,BALLAST_STATE,VALVE_QUEUE,BALLAST_QUEUE,NUM_VALVES,NUM_BALLASTS,NUM_VALVE_ATTEMPTS,NUM_BALLAST_ATTEMPTS,CUTDOWN_STATE,PRESS,TEMP,JOULES,VOLTAGE,CURRENT,CURRENT_GPS,CURRENT_RB,CURRENT_MOTORS,CURRENT_PAYLOAD,TEMP_NECK,SPEED_GPS,HEADING_GPS,NUM_SATS_GPS,LOOP_TIME,RB_SENT_COMMS,COMMS_INTERVAL,GPS_INTERVAL,TEMP_SETPOINT,MANUAL_MODE,RB_SHOULD_USE,GPS_SHOULD_USE,HEATER_SHOULD_USE,HEATER_STRONG_ENABLE,HEATER_WEEK_ENABLE,BAT_GOOD_STATE,CURR_GOOD_STATE,PRES_GOOD_STATE,TEMP_GOOD_STATE,RB_GOOD_STATE,GPS_GOOD_STATE,PRESS_BASELINE,DO_NOTHING_INTERVAL,INCENTIVE_THRESHOLD,RE_ARM_CONSTANT,BALLAST_ARM_ALT,VALVE_SETPOINT,VALVE_DURATION,VALVE_ALT_LAST,VALVE_VELOCITY_CONSTANT,VALVE_ALTITUDE_DIFF_CONSTANT,VALVE_LAST_ACTION_CONSTANT,BALLAST_SETPOINT,BALLAST_DURATION,BALLAST_ALT_LAST,BALLAST_VELOCITY_CONSTANT,BALLAST_ALTITUDE_DIFF_CONSTANT,BALLAST_LAST_ACTION_CONSTANT,SHOULD_CUTDOWN,SHOULD_LED,SETUP_STATE,DEBUG_STATE,FORCE_VALVE,FORCE_BALLAST,REPORT_MODE,BMP_1_ENABLE,BMP_2_ENABLE,BMP_3_ENABLE,BMP_4_ENABLE,RAW_TEMP_1,RAW_TEMP_2,RAW_TEMP_3,RAW_TEMP_4,RAW_PRESSURE_1,RAW_PRESSURE_2,RAW_PRESSURE_3,RAW_PRESSURE_4,ALTITUDE_LAST,GPS_LAST,COMMS_LAST,COMMS_LENGTH";

static const bool      CUTDOWN_ALT_ENABLE            =                    false;
static const bool      CUTDOWN_GPS_ENABLE            =                    false;
static const uint16_t  CUTDOWN_ALT                   =                    30000;
static const uint16_t  CUTDOWN_TIME                  =                    10000;

static const uint16_t  GPS_LOCK_TIME                 =                      500;
static const float     GPS_FENCE_LAT_MIN             =                    -9999;
static const float     GPS_FENCE_LAT_MAX             =                     9999;
static const float     GPS_FENCE_LON_MIN             =                    -9999;
static const float     GPS_FENCE_LON_MAX             =                     9999;

static const uint32_t  GPS_INTERVAL_DEFAULT          =                    12000;
static const uint32_t  COMMS_INTERVAL_DEFAULT        =                   120000;
static const uint32_t  COMMS_RESTART_INTERVAL        =                  1800000;
static const uint32_t  COMMS_DEBUG_INTERVAL          =                    60000;
static const uint16_t  LOOP_INTERVAL                 =                       50;

static const uint16_t  DEBUG_ALT                     =                      300;
static const uint16_t  BUFFER_SIZE                   =                      200;
static const uint32_t  FILE_RESET_TIME               =                432000000;
static const uint32_t  CONSOLE_BAUD                  =                   115200;
static const uint32_t  GPS_BAUD                      =                     9600;
static const uint32_t  RB_BAUD                       =                    19200;

/***************************  Flight Parameters  ******************************/
static const double    TEMP_SETPOINT_DEFAULT         =                        0;
static const bool      MANUAL_MODE_DEFAULT           =                     true;

static const double    PRESS_BASELINE_DEFAULT        =                 101325.0;
static const uint32_t  DO_NOTHING_INTERVAL_DEFAULT   =                    30000;
static const float     INCENTIVE_THRESHOLD_DEFAULT   =                     0.75;
static const float     RE_ARM_DEFAULT                =                        0;
static const float     BALLAST_ARM_ALT_DEFAULT       =                  13000.0;

static const float     VALVE_SETPOINT_DEFAULT        =                  13500.0;
static const uint16_t  VALVE_DURATION_DEFAULT        =                    20000;
static const float     VALVE_ALT_LAST_DEFAULT        =                        0;
static const float     VALVE_VELOCITY_DEFAULT        =                      1.0;
static const float     VALVE_ALTITUDE_DIFF_DEFAULT   =               1.0/1000.0;
static const float     VALVE_LAST_ACTION_DEFAULT     =               1.0/1000.0;

static const float     BALLAST_SETPOINT_DEFAULT      =                  13000.0;
static const uint16_t  BALLAST_DURATION_DEFAULT      =                    20000;
static const float     BALLAST_ALT_LAST_DEFAULT      =                 -90000.0;
static const float     BALLAST_VELOCITY_DEFAULT      =                      1.0;
static const float     BALLAST_ALTITUDE_DIFF_DEFAULT =               1.0/1000.0;
static const float     BALLAST_LAST_ACTION_DEFAULT   =               1.0/1000.0;

static const uint16_t  VALVE_OPENING_TIMEOUT         =                     1000;
static const uint16_t  VALVE_CLOSING_TIMEOUT         =                     2500;
static const uint16_t  VALVE_CUTDOWN_TIMEOUT         =                    10000;
static const uint16_t  VALVE_LEAK_TIMEOUT            =                    20000;
static const uint16_t  BALLAST_REVERSE_TIMEOUT       =                    20000;
static       uint16_t  VALVE_MOTOR_SPEED             =                      255;
static       uint16_t  BALLAST_MOTOR_SPEED           =                      255;

/*****************************  EEPROM CONSTANTS  *****************************/
static const uint8_t   EEPROM_CLEAR_NUM              =                        8;

static const uint8_t   EEPROM_VALVE_START            =                        0;
static const uint8_t   EEPROM_VALVE_END              =                        4;
static const uint8_t   EEPROM_BALLAST_START          =                        5;
static const uint8_t   EEPROM_BALLAST_END            =                        9;

static const uint8_t   EEPROM_ROCKBLOCK              =                       10;
static const uint8_t   EEPROM_GPS                    =                       11;
static const uint8_t   EEPROM_HEATER                 =                       12;

#endif
