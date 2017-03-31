/*
  Stanford Student Space Initiative
  Balloons | VALBAL | March 2017
  Davy Ragland | dragland@stanford.edu

  File: data.h
  --------------------------
  Data frame structure.
*/

#ifndef data_H
#define data_H

#include "Config.h"

/**************************  CURRENT DATA FRAME  ******************************/
struct DataFrame {
/******************************  PRIMARY DATA  ********************************/
  uint32_t     TIME                            =                              0;
  float        LAT_GPS                         =                              0;
  float        LONG_GPS                        =                              0;
  double       ALTITUDE                        =                              0;
  double       ALTITUDE_GPS                    =                              0;
  double       ASCENT_RATE                     =                              0;
  float        VALVE_INCENTIVE                 =                              0;
  float        BALLAST_INCENTIVE               =                              0;
  bool         VALVE_STATE                     =                          false;
  bool         BALLAST_STATE                   =                          false;
  uint32_t     NUM_VALVES                      =                              0;
  uint32_t     NUM_BALLASTS                    =                              0;
  bool         CUTDOWN_STATE                   =                          false;

  double       PRESS                           =                              0;
  double       TEMP                            =                              0;
  double       JOULES                          =                              0;
  double       VOLTAGE                         =                              0;
  double       CURRENT                         =                              0;
  double       CURRENT_GPS                     =                              0;
  double       CURRENT_RB                      =                              0;
  double       CURRENT_MOTORS                  =                              0;
  double       CURRENT_PAYLOAD                 =                              0;
  double       TEMP_NECK                       =                              0;
  double       SPEED_GPS                       =                              0;
  double       HEADING_GPS                     =                              0;
  uint32_t     NUM_SATS_GPS                    =                              0;
  uint16_t     LOOP_TIME                       =                              0;
  uint16_t     RB_SENT_COMMS                   =                              0;

  uint32_t     COMMS_INTERVAL                  =         COMMS_INTERVAL_DEFAULT;
  uint32_t     GPS_INTERVAL                    =           GPS_INTERVAL_DEFAULT;
  double       TEMP_SETPOINT                   =          TEMP_SETPOINT_DEFAULT;
  bool         MANUAL_MODE                     =            MANUAL_MODE_DEFAULT;

  bool         RB_SHOULD_USE                   =                           true;
  bool         GPS_SHOULD_USE                  =                           true;
  bool         HEATER_SHOULD_USE               =                           true;
  bool         HEATER_STRONG_ENABLE            =                           true;
  bool         HEATER_WEEK_ENABLE              =                          false;

  bool         BAT_GOOD_STATE                  =                          false;
  bool         CURR_GOOD_STATE                 =                          false;
  bool         PRES_GOOD_STATE                 =                          false;
  bool         TEMP_GOOD_STATE                 =                          false;
  bool         RB_GOOD_STATE                   =                          false;
  bool         GPS_GOOD_STATE                  =                          false;

/*****************************  SECONDARY DATA  *******************************/
  double       PRESS_BASELINE                  =         PRESS_BASELINE_DEFAULT;
  uint32_t     DO_NOTHING_INTERVAL             =    DO_NOTHING_INTERVAL_DEFAULT;
  float        INCENTIVE_THRESHOLD             =    INCENTIVE_THRESHOLD_DEFAULT;
  float        RE_ARM_CONSTANT                 =                 RE_ARM_DEFAULT;
  float        BALLAST_ARM_ALT                 =        BALLAST_ARM_ALT_DEFAULT;

  float        VALVE_SETPOINT                  =         VALVE_SETPOINT_DEFAULT;
  uint16_t     VALVE_DURATION                  =         VALVE_DURATION_DEFAULT;
  double       VALVE_ALT_LAST                  =         VALVE_ALT_LAST_DEFAULT;
  float        VALVE_VELOCITY_CONSTANT         =         VALVE_VELOCITY_DEFAULT;
  float        VALVE_ALTITUDE_DIFF_CONSTANT    =    VALVE_ALTITUDE_DIFF_DEFAULT;
  float        VALVE_LAST_ACTION_CONSTANT      =      VALVE_LAST_ACTION_DEFAULT;

  float        BALLAST_SETPOINT                =       BALLAST_SETPOINT_DEFAULT;
  uint16_t     BALLAST_DURATION                =       BALLAST_DURATION_DEFAULT;
  double       BALLAST_ALT_LAST                =       BALLAST_ALT_LAST_DEFAULT;
  float        BALLAST_VELOCITY_CONSTANT       =       BALLAST_VELOCITY_DEFAULT;
  float        BALLAST_ALTITUDE_DIFF_CONSTANT  =  BALLAST_ALTITUDE_DIFF_DEFAULT;
  float        BALLAST_LAST_ACTION_CONSTANT    =    BALLAST_LAST_ACTION_DEFAULT;

  bool         BMP_1_ENABLE                    =                           true;
  bool         BMP_2_ENABLE                    =                           true;
  bool         BMP_3_ENABLE                    =                           true;
  bool         BMP_4_ENABLE                    =                           true;

  bool         SHOULD_CUTDOWN                  =                          false;
  bool         SHOULD_LED                      =                          false;
  bool         SETUP_STATE                     =                           true;
  bool         DEBUG_STATE                     =                           true;
  bool         FORCE_VALVE                     =                          false;
  bool         FORCE_BALLAST                   =                          false;
  bool         REPORT_MODE                     =                          false;

  double       ALTITUDE_LAST                   =                              0;
  uint64_t     GPS_LAST                        =                              0;
  uint64_t     COMMS_LAST                      =                              0;
  uint16_t     COMMS_LENGTH                    =                              0;

  double       RAW_TEMP_1                      =                              0;
  double       RAW_TEMP_2                      =                              0;
  double       RAW_TEMP_3                      =                              0;
  double       RAW_TEMP_4                      =                              0;

  double       RAW_PRESSURE_1                  =                              0;
  double       RAW_PRESSURE_2                  =                              0;
  double       RAW_PRESSURE_3                  =                              0;
  double       RAW_PRESSURE_4                  =                              0;

  double       ALTITUDE_1                      =                              0;
  double       ALTITUDE_2                      =                              0;
  double       ALTITUDE_3                      =                              0;
  double       ALTITUDE_4                      =                              0;
};

#endif
