/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
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
  uint32_t   TIME                            =                                0;
  uint32_t   LOOP_NUMBER                     =                                0; // Useful for checking data integrity
  float      LAT_GPS                         =                                0;
  float      LONG_GPS                        =                                0;
  float      ALTITUDE_BAROMETER              =                                0;
  float      ALTITUDE_GPS                    =                                0;
  float      ASCENT_RATE                     =                                0;
  float      VALVE_INCENTIVE                 =                                0;
  float      BALLAST_INCENTIVE               =                                0;
  bool       VALVE_STATE                     =                            false;
  bool       BALLAST_STATE                   =                            false;
  uint32_t   VALVE_QUEUE                     =                                0;
  uint32_t   BALLAST_QUEUE                   =                                0;
  uint32_t   VALVE_TIME_TOTAL                =                                0;
  uint32_t   BALLAST_TIME_TOTAL              =                                0;
  uint32_t   VALVE_NUM_ACTIONS               =                                0;
  uint32_t   BALLAST_NUM_ACTIONS             =                                0;
  uint32_t   VALVE_NUM_ATTEMPTS              =                                0;
  uint32_t   BALLAST_NUM_ATTEMPTS            =                                0;
  uint32_t   BALLAST_NUM_OVERCURRENTS        =                                0;
  bool       CUTDOWN_STATE                   =                            false;

  float      TEMP_INT                        =                                0;
  float      JOULES_TOTAL                    =                                0;
  float      VOLTAGE_PRIMARY                 =                                0;
  float      VOLTAGE_SUPERCAP                =                                0;
  float      CURRENT_TOTAL_AVG               =                                0;
  float      CURRENT_TOTAL_MIN               =                                0;
  float      CURRENT_TOTAL_MAX               =                                0;
  float      CURRENT_RB_AVG                  =                                0;
  float      CURRENT_RB_MAX                  =                                0;
  float      CURRENT_MOTOR_VALVE_AVG         =                                0;
  float      CURRENT_MOTOR_VALVE_MAX         =                                0;
  float      CURRENT_MOTOR_BALLAST_AVG       =                                0;
  float      CURRENT_MOTOR_BALLAST_MAX       =                                0;
  float      CURRENT_PAYLOAD_AVG             =                                0;
  float      CURRENT_PAYLOAD_MAX             =                                0;
  float      TEMP_EXT                        =                                0;
  uint32_t   LOOP_TIME_MAX                   =                                0;
  uint32_t   RB_SENT_COMMS                   =                                0;
  uint32_t   RB_WAKE_FAILS                   =                                0;
  uint32_t   RB_SLEEP_FAILS                  =                                0;
  uint8_t    RESISTOR_MODE                   =                                0;                

  bool       MANUAL_MODE                     =              MANUAL_MODE_DEFAULT;
  uint8_t    REPORT_MODE                     =              REPORT_MODE_DEFAULT;
  bool       SHOULD_REPORT                   =                            false;

/*****************************  SECONDARY DATA  *******************************/
  bool       POWER_STATE_LED                 =                             true;
  bool       POWER_STATE_RB                  =                             true;
  bool       POWER_STATE_GPS                 =                             true;
  bool       POWER_STATE_PAYLOAD             =                             true;
  uint8_t    NUM_SATS_GPS                    =                                0;
  float      SPEED_GPS                       =                                0;
  float      HEADING_GPS                     =                                0;

  float      INCENTIVE_NOISE                 =          INCENTIVE_NOISE_DEFAULT;
  float      RE_ARM_CONSTANT                 =                   RE_ARM_DEFAULT;
  float      VALVE_ALT_LAST                  =           VALVE_ALT_LAST_DEFAULT;
  float      BALLAST_ALT_LAST                =         BALLAST_ALT_LAST_DEFAULT;

  bool       DEBUG_STATE                     =                             true;
  bool       FORCE_VALVE                     =                            false;
  bool       FORCE_BALLAST                   =                            false;

  bool       BMP_1_ENABLE                    =                             true;
  bool       BMP_2_ENABLE                    =                             true;
  bool       BMP_3_ENABLE                    =                             true;
  bool       BMP_4_ENABLE                    =                             true;
  uint32_t   BMP_1_REJECTIONS                =                                0;
  uint32_t   BMP_2_REJECTIONS                =                                0;
  uint32_t   BMP_3_REJECTIONS                =                                0;
  uint32_t   BMP_4_REJECTIONS                =                                0;

  float      BLACK_BODY_TEMP                 =                                0;

/*****************************  TERTIARY DATA  ********************************/
  uint32_t   RB_INTERVAL                     =              RB_INTERVAL_DEFAULT;
  uint32_t   GPS_INTERVAL                    =             GPS_INTERVAL_DEFAULT;
  bool       RB_SHOULD_SLEEP                 =          RB_SHOULD_SLEEP_DEFAULT;

  float      PRESS_BASELINE                  =           PRESS_BASELINE_DEFAULT;
  float      INCENTIVE_THRESHOLD             =      INCENTIVE_THRESHOLD_DEFAULT;
  float      BALLAST_ARM_ALT                 =          BALLAST_ARM_ALT_DEFAULT;

  uint32_t   BALLAST_REVERSE_INTERVAL        = BALLAST_REVERSE_INTERVAL_DEFAULT;
  uint32_t   VALVE_LEAK_INTERVAL             =      VALVE_LEAK_INTERVAL_DEFAULT;
  uint16_t   BALLAST_STALL_CURRENT           =    BALLAST_STALL_CURRENT_DEFAULT;
  uint16_t   VALVE_MOTOR_SPEED_OPEN          =   VALVE_MOTOR_SPEED_OPEN_DEFAULT;
  uint16_t   VALVE_MOTOR_SPEED_CLOSE         =  VALVE_MOTOR_SPEED_CLOSE_DEFAULT;
  uint16_t   BALLAST_MOTOR_SPEED             =      BALLAST_MOTOR_SPEED_DEFAULT;
  uint32_t   VALVE_OPENING_DURATION          =   VALVE_OPENING_DURATION_DEFAULT;
  uint32_t   VALVE_CLOSING_DURATION          =   VALVE_CLOSING_DURATION_DEFAULT;

  float      VALVE_SETPOINT                  =           VALVE_SETPOINT_DEFAULT;
  uint32_t   VALVE_VENT_DURATION             =      VALVE_VENT_DURATION_DEFAULT;
  uint32_t   VALVE_FORCE_DURATION            =      VALVE_VENT_DURATION_DEFAULT;
  float      VALVE_VELOCITY_CONSTANT         =           VALVE_VELOCITY_DEFAULT;
  float      VALVE_ALTITUDE_DIFF_CONSTANT    =      VALVE_ALTITUDE_DIFF_DEFAULT;
  float      VALVE_LAST_ACTION_CONSTANT      =        VALVE_LAST_ACTION_DEFAULT;

  float      BALLAST_SETPOINT                =         BALLAST_SETPOINT_DEFAULT;
  uint32_t   BALLAST_DROP_DURATION           =    BALLAST_DROP_DURATION_DEFAULT;
  uint32_t   BALLAST_FORCE_DURATION          =    BALLAST_DROP_DURATION_DEFAULT;
  float      BALLAST_VELOCITY_CONSTANT       =         BALLAST_VELOCITY_DEFAULT;
  float      BALLAST_ALTITUDE_DIFF_CONSTANT  =    BALLAST_ALTITUDE_DIFF_DEFAULT;
  float      BALLAST_LAST_ACTION_CONSTANT    =      BALLAST_LAST_ACTION_DEFAULT;

/*******************************  EXTRA DATA  *********************************/
  bool       SETUP_STATE                     =                             true;
  bool       SHOULD_CUTDOWN                  =                            false;

  uint32_t   LOOP_TIME                       =                                0;
  uint32_t   LOG_TIME                        =                                0;
  float      RAW_TEMP_1                      =                                0;
  float      RAW_TEMP_2                      =                                0;
  float      RAW_TEMP_3                      =                                0;
  float      RAW_TEMP_4                      =                                0;
  float      RAW_PRESSURE_1                  =                                0;
  float      RAW_PRESSURE_2                  =                                0;
  float      RAW_PRESSURE_3                  =                                0;
  float      RAW_PRESSURE_4                  =                                0;
  float      PRESS                           =                                0;
  float      CURRENT_TOTAL                   =                                0;
  float      CURRENT_RB                      =                                0;
  float      CURRENT_MOTOR_VALVE             =                                0;
  float      CURRENT_MOTOR_BALLAST           =                                0;
  float      CURRENT_PAYLOAD                 =                                0;
  uint32_t   PAYLOAD_MESSAGE_SIZE            =                                0;

  uint32_t   GPS_LAST                        =                                0;
  uint32_t   RB_LAST                         =                                0;
  uint32_t   DATAFILE_LAST                   =                                0;
  uint16_t   COMMS_LENGTH                    =                                0;
  uint32_t   LOOP_NUMBER2                    =                                0;
} __attribute__((packed));

#endif
