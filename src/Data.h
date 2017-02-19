/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu

  File: data.h
  --------------------------
  Data frame structure.
*/

#ifndef data_H
#define data_H

#include "Config.h"

/**********************************  DATA  ************************************/
struct DataFrame {
  uint32_t     TIME                           =          0;
  double       MINUTES                        =          0;
  uint16_t     LOOP_RATE                      =          0;
  double       VOLTAGE                        =          0;
  double       CURRENT                        =          0;
  double       CURRENT_GPS                    =          0;
  double       CURRENT_RB                     =          0;
  double       CURRENT_MOTORS                 =          0;
  double       CURRENT_PAYLOAD                =          0;
  double       ALTITUDE_BMP                   =          0;
  double       ASCENT_RATE                    =          0;
  double       TEMP                           =          0;
  float        LAT_GPS                        =          0;
  float        LONG_GPS                       =          0;
  double       SPEED_GPS                      =          0;
  double       HEADING_GPS                    =          0;
  double       ALTITUDE_GPS                   =          0;
  double       PRESS_BMP                      =          0;
  uint32_t     NUM_SATS_GPS                   =          0;
  uint16_t     RB_SENT_COMMS                  =          0;
  bool         CUTDOWN_STATE                  =      false;

  bool         BAT_GOOD_STATE                 =      false;
  bool         CURR_GOOD_STATE                =      false;
  bool         PRES_GOOD_STATE                =      false;
  bool         TEMP_GOOD_STATE                =      false;
  bool         CAN_GOOD_STATE                 =      false;
  bool         RB_GOOD_STATE                  =      false;
  bool         GPS_GOOD_STATE                 =      false;
  bool         LOOP_GOOD_STATE                =      false;

  bool         VALVE_STATE                    =      false;
  bool         BALLAST_STATE                  =      false;
  float        VALVE_INCENTIVE                =        0.0;
  float        BALLAST_INCENTIVE              =        0.0;
  float        VALVE_SETPOINT                 =    13500.0;
  float        BALLAST_SETPOINT               =    13000.0;
  float        VALVE_VELOCITY_CONSTANT        =        1.0;
  float        VALVE_ALTITUDE_DIFF_CONSTANT   = 1.0/1000.0;
  float        VALVE_LAST_ACTION_CONSTANT     = 1.0/1000.0;
  float        BALLAST_VELOCITY_CONSTANT      =        1.0;
  float        BALLAST_ALTITUDE_DIFF_CONSTANT = 1.0/1000.0;
  float        BALLAST_LAST_ACTION_CONSTANT   = 1.0/1000.0;

  bool         SETUP_STATE                    =       true;
  bool         DEBUG_STATE                    =       true;
  bool         SHOULD_CUTDOWN                 =      false;
  bool         FORCE_VALVE                    =      false;
  bool         FORCE_BALLAST                  =      false;
  double       ALTITUDE_LAST                  =          0;
  double       VALVE_ALT_LAST                 =          0;
  double       BALLAST_ALT_LAST               =   -90000.0;
  uint64_t     COMMS_LAST                     =          0;
  uint64_t     LOOP_START                     =          0;
  uint16_t     COMMS_LENGTH                   =          0;
};

#endif
