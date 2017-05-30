/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
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
  uint32_t   TIME                             =                               0;
  float      LAT_GPS                          =                               0;
  float      LONG_GPS                         =                               0;
  double     ALTITUDE                         =                               0;
  double     ALTITUDE_GPS                     =                               0;
  double     ASCENT_RATE                      =                               0;
  float      VALVE_INCENTIVE                  =                               0;
  float      BALLAST_INCENTIVE                =                               0;
  bool       VALVE_STATE                      =                           false;
  bool       BALLAST_STATE                    =                           false;
  uint32_t   VALVE_QUEUE                      =                               0;
  uint32_t   BALLAST_QUEUE                    =                               0;
  uint32_t   VALVE_TIME_TOTAL                 =                               0;
  uint32_t   BALLAST_TIME_TOTAL               =                               0;
  uint32_t   NUM_VALVES                       =                               0;
  uint32_t   NUM_BALLASTS                     =                               0;
  uint32_t   NUM_VALVE_ATTEMPTS               =                               0;
  uint32_t   NUM_BALLAST_ATTEMPTS             =                               0;
  uint32_t   NUM_BALLAST_OVER_CURRENTS        =                               0;
  bool       CUTDOWN_STATE                    =                           false;

  double     TEMP_IN                          =                               0;
  double     JOULES                           =                               0;
  double     VOLTAGE                          =                               0;
  double     CURRENT_AVG                      =                               0;
  double     CURRENT_MIN                      =                               0;
  double     CURRENT_MAX                      =                               0;
  double     CURRENT_GPS_AVG                  =                               0;
  double     CURRENT_GPS_MAX                  =                               0;
  double     CURRENT_RB_AVG                   =                               0;
  double     CURRENT_RB_MAX                   =                               0;
  double     CURRENT_MOTORS_AVG               =                               0;
  double     CURRENT_MOTORS_MAX               =                               0;
  double     CURRENT_PAYLOAD_AVG              =                               0;
  double     CURRENT_PAYLOAD_MAX              =                               0;
  double     TEMP_EXT                         =                               0;
  uint32_t   LOOP_TIME                        =                               0;
  uint32_t   LOOP_TIME_MAX                    =                               0;
  uint32_t   RB_SENT_COMMS                    =                               0;

  float      EULER_X_AVG                      =                               0;
  float      EULER_Y_AVG                      =                               0;
  float      EULER_Z_AVG                      =                               0;

  bool       MANUAL_MODE                      =             MANUAL_MODE_DEFAULT;
  uint8_t    REPORT_MODE                      =             REPORT_MODE_DEFAULT;
  bool       SHOULD_REPORT                    =                           false;

/*****************************  SECONDARY DATA  *******************************/
  bool       RB_SHOULD_USE                    =                            true;
  bool       GPS_SHOULD_USE                   =                            true;
  bool       HEATER_SHOULD_USE                =                            true;
  bool       PAYLOAD_SHOULD_USE               =                            true;
  bool       HEATER_STRONG_ENABLE             =                            true;
  bool       HEATER_WEEK_ENABLE               =                           false;
  bool       GPS_GOOD_STATE                   =                           false;
  double     SPEED_GPS                        =                               0;
  double     HEADING_GPS                      =                               0;
  uint32_t   NUM_SATS_GPS                     =                               0;

  float      INCENTIVE_NOISE                  =         INCENTIVE_NOISE_DEFAULT;
  float      RE_ARM_CONSTANT                  =                  RE_ARM_DEFAULT;
  double     VALVE_ALT_LAST                   =          VALVE_ALT_LAST_DEFAULT;
  double     BALLAST_ALT_LAST                 =        BALLAST_ALT_LAST_DEFAULT;

  bool       SHOULD_LED                       =                            true;
  bool       DEBUG_STATE                      =                            true;
  bool       FORCE_VALVE                      =                           false;
  bool       FORCE_BALLAST                    =                           false;
  bool       BMP_1_ENABLE                     =                            true;
  bool       BMP_2_ENABLE                     =                            true;
  bool       BMP_3_ENABLE                     =                            true;
  bool       BMP_4_ENABLE                     =                            true;
  uint32_t   BMP_1_REJECTIONS                 =                               0;
  uint32_t   BMP_2_REJECTIONS                 =                               0;
  uint32_t   BMP_3_REJECTIONS                 =                               0;
  uint32_t   BMP_4_REJECTIONS                 =                               0;

/*****************************  TERTIARY DATA  ********************************/
  double     TEMP_SETPOINT                    =           TEMP_SETPOINT_DEFAULT;
  uint32_t   COMMS_INTERVAL                   =          COMMS_INTERVAL_DEFAULT;
  uint32_t   GPS_INTERVAL                     =            GPS_INTERVAL_DEFAULT;

  double     PRESS_BASELINE                   =          PRESS_BASELINE_DEFAULT;
  float      INCENTIVE_THRESHOLD              =     INCENTIVE_THRESHOLD_DEFAULT;
  float      BALLAST_ARM_ALT                  =         BALLAST_ARM_ALT_DEFAULT;

  uint32_t   BALLAST_REVERSE_TIMEOUT          = BALLAST_REVERSE_TIMEOUT_DEFAULT;
  uint16_t   BALLAST_STALL_CURRENT            =   BALLAST_STALL_CURRENT_DEFAULT;
  uint16_t   VALVE_MOTOR_SPEED                =       VALVE_MOTOR_SPEED_DEFAULT;
  uint16_t   BALLAST_MOTOR_SPEED              =     BALLAST_MOTOR_SPEED_DEFAULT;
  uint32_t   VALVE_OPENING_TIMEOUT            =   VALVE_OPENING_TIMEOUT_DEFAULT;
  uint32_t   VALVE_CLOSING_TIMEOUT            =   VALVE_CLOSING_TIMEOUT_DEFAULT;

  float      VALVE_SETPOINT                   =          VALVE_SETPOINT_DEFAULT;
  uint32_t   VALVE_DURATION                   =          VALVE_DURATION_DEFAULT;
  uint32_t   VALVE_FORCE_DURATION             =          VALVE_DURATION_DEFAULT;
  float      VALVE_VELOCITY_CONSTANT          =          VALVE_VELOCITY_DEFAULT;
  float      VALVE_ALTITUDE_DIFF_CONSTANT     =     VALVE_ALTITUDE_DIFF_DEFAULT;
  float      VALVE_LAST_ACTION_CONSTANT       =       VALVE_LAST_ACTION_DEFAULT;

  float      BALLAST_SETPOINT                 =        BALLAST_SETPOINT_DEFAULT;
  uint32_t   BALLAST_DURATION                 =        BALLAST_DURATION_DEFAULT;
  uint32_t   BALLAST_FORCE_DURATION           =        BALLAST_DURATION_DEFAULT;
  float      BALLAST_VELOCITY_CONSTANT        =        BALLAST_VELOCITY_DEFAULT;
  float      BALLAST_ALTITUDE_DIFF_CONSTANT   =   BALLAST_ALTITUDE_DIFF_DEFAULT;
  float      BALLAST_LAST_ACTION_CONSTANT     =     BALLAST_LAST_ACTION_DEFAULT;

/*******************************  EXTRA DATA  *********************************/
  bool       SETUP_STATE                      =                            true;
  bool       SHOULD_CUTDOWN                   =                           false;

  double     RAW_TEMP_1                       =                               0;
  double     RAW_TEMP_2                       =                               0;
  double     RAW_TEMP_3                       =                               0;
  double     RAW_TEMP_4                       =                               0;
  double     RAW_PRESSURE_1                   =                               0;
  double     RAW_PRESSURE_2                   =                               0;
  double     RAW_PRESSURE_3                   =                               0;
  double     RAW_PRESSURE_4                   =                               0;
  double     PRESS                            =                               0;
  double     CURRENT                          =                               0;
  double     CURRENT_GPS                      =                               0;
  double     CURRENT_RB                       =                               0;
  double     CURRENT_MOTORS                   =                               0;
  double     CURRENT_PAYLOAD                  =                               0;
  float      EULER_X                          =                               0;
  float      EULER_Y                          =                               0;
  float      EULER_Z                          =                               0;

  uint32_t   GPS_LAST                         =                               0;
  uint32_t   COMMS_LAST                       =                               0;
  uint32_t   DATAFILE_LAST                    =                               0;
  uint16_t   COMMS_LENGTH                     =                               0;
};

#endif
