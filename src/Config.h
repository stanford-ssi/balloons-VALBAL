/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
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

/****************************  EDITABLE CONSTANTS  ****************************/
static const char      MISSION_NUMBER[]        = "SSI-51";
static const char      CSV_DATA_HEADER[]       = "TIME,LOOP_RATE,VOLTAGE,CURRENT,ALTITUDE_BMP,ASCENT_RATE,TEMP_IN,LAT_GPS,LONG_GPS,SPEED_GPS,HEADING_GPS,ALTITUDE_GPS,PRESS_BMP,RB_SENT_COMMS,CUTDOWN_STATE";

static const bool      CUTDOWN_ALT_ENABLE      =   true;
static const bool      CUTDOWN_GPS_ENABLE      =   true;
static const uint16_t  CUTDOWN_ALT             =  20000;
static const uint16_t  CUTDOWN_TIME            =   5000;

static const uint16_t  GPS_LOCK_TIME           =    500;
static const float     GPS_FENCE_LAT_MIN       =  -9999;
static const float     GPS_FENCE_LAT_MAX       =   9999;
static const float     GPS_FENCE_LON_MIN       =  -9999;
static const float     GPS_FENCE_LON_MAX       =   9999;

static const uint16_t  COMMS_RATE              =  60000;
static const uint16_t  LOOP_RATE               =     50;

static const uint16_t  DEBUG_ALT               =    300;
static const uint16_t  ANALOG_RES              =     12;
static const uint16_t  ANALOG_MAX              =   4095;
static const uint16_t  BUFFER_SIZE             =    200;
static const uint32_t  FILE_RESET_TIME         =   7200;
static const uint32_t  CONSOLE_BAUD            = 115200;
static const uint32_t  GPS_BAUD                =   9600;
static const uint32_t  RB_BAUD                 =  19200;
static const double    PID_SETPOINT            =      0;


/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   REBOOT_ENABLE           =      0;
static const uint8_t   SD_CS                   =     10;
static const uint8_t   FAULT_PIN               =     26;
static const uint8_t   BMP_CS_ONE              =      9;
static const uint8_t   BMP_CS_TWO              =     15;
static const uint8_t   BMP_CS_THREE            =      5;
static const uint8_t   BMP_CS_FOUR             =     32;
static const uint8_t   VBAT_PIN                =    A14;
static const uint8_t   BATT_CURRENT            =    A10;
static const uint8_t   EXTERNAL_CURRENT        =    A11;
static const uint8_t   VALVE_REVERSE           =      6;
static const uint8_t   VALVE_FORWARD           =      0;
static const uint8_t   BALLAST_REVERSE         =      5;
static const uint8_t   BALLAST_FORWARD         =     21;
static const uint8_t   HEATER_INTERNAL_STRONG  =      4;
static const uint8_t   HEATER_INTERNAL_WEAK    =      3;
static const uint8_t   GPS_ENABLE              =     17;
static const uint8_t   RB_GATE                 =     28;
static const uint8_t   RB_SLEEP                =     14;
static const uint8_t   PAYLOAD_GATE            =     31;

#endif
