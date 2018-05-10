  uint32_t   TIME                            =                                0;
  uint32_t   LOOP_NUMBER                     =                                0; // Useful for checking data integrity
  uint32_t CPU_SPEED = F_CPU;
  float      LAT_GPS                         =                                0;
  float      LONG_GPS                        =                                0;
  float      ALTITUDE_BAROMETER              =                                0;
  float      ALTITUDE_GPS                    =                                0;
  float      ASCENT_RATE                     =                                0;
  int32_t    ACTION                          =                                0;
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
  uint8_t    MAX_CURRENT_CHARGING_LIMIT      =                                0;
  uint8_t    SYSTEM_POWER_STATE              =                                0;

  float      TEMP_INT                        =                                0;
  float      JOULES_TOTAL                    =                                0;
  float      VOLTAGE_PRIMARY                 =                                0;
  float      VOLTAGE_SUPERCAP_AVG            =                                0;
  float      CURRENT_TOTAL_AVG               =                                0;
  float      CURRENT_TOTAL_MIN               =                                0;
  float      CURRENT_TOTAL_MAX               =                                0;
  float      CURRENT_RB_AVG                  =                                0;
  float      CURRENT_RB_MAX                  =                                0;
  float      CURRENT_MOTORS                  =                                0;
  float      CURRENT_MOTOR_VALVE_AVG         =                                0;
  float      CURRENT_MOTOR_VALVE_MAX         =                                0;
  float      CURRENT_MOTOR_BALLAST_AVG       =                                0;
  float      CURRENT_MOTOR_BALLAST_MAX       =                                0;
  float      CURRENT_PAYLOAD_AVG             =                                0;
  float      CURRENT_PAYLOAD_MAX             =                                0;
  float      TEMP_EXT                        =                                0;
  uint32_t   LOOP_TIME_MAX                   =                                0;
  uint32_t   RB_SENT_COMMS                   =                                0;
  uint8_t    RESISTOR_MODE                   =                                0;

  bool       MANUAL_MODE                     =              MANUAL_MODE_DEFAULT;
  uint8_t    REPORT_MODE                     =              REPORT_MODE_DEFAULT;
  bool       SHOULD_REPORT                   =                            false;

  bool       POWER_STATE_LED                 =                             true;
  bool       POWER_STATE_RB                  =                             true;
  bool       POWER_STATE_GPS                 =                             false;
  bool       POWER_STATE_PAYLOAD             =                             true;
  uint8_t    NUM_SATS_GPS                    =                                0;
  float      SPEED_GPS                       =                                0;
  float      HEADING_GPS                     =                                0;

  float      INCENTIVE_NOISE                 =          INCENTIVE_NOISE_DEFAULT;
  float      RE_ARM_CONSTANT                 =                   RE_ARM_DEFAULT;
  float      VALVE_ALT_LAST                  =           VALVE_ALT_LAST_DEFAULT;
  float      BALLAST_ALT_LAST                =         BALLAST_ALT_LAST_DEFAULT;

  uint8_t    CURRENT_CONTROLLER_INDEX        =         CONTROLLER_INDEX_DEFAULT;

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

  uint32_t   RB_INTERVAL                     =              RB_INTERVAL_DEFAULT;
  uint32_t   GPS_INTERVAL                    =             GPS_INTERVAL_DEFAULT;

  float      PRESS_BASELINE                  =           PRESS_BASELINE_DEFAULT;
  float      INCENTIVE_THRESHOLD             =      INCENTIVE_THRESHOLD_DEFAULT;
  float      BALLAST_ARM_ALT                 =          BALLAST_ARM_ALT_DEFAULT;

  uint32_t   BALLAST_REVERSE_INTERVAL        = BALLAST_REVERSE_INTERVAL_DEFAULT;
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
  float      VOLTAGE_SUPERCAP                =                                0;
  float      CURRENT_TOTAL                   =                                0;
  float      CURRENT_RB                      =                                0;
  float      CURRENT_MOTOR_VALVE             =                                0;
  float      CURRENT_MOTOR_BALLAST           =                                0;
  float      CURRENT_PAYLOAD                 =                                0;

  uint32_t   GPS_LAST                        =                                0;
  uint32_t   RB_LAST                         =                                0;
  uint32_t   DATAFILE_LAST                   =                                0;
  uint16_t   COMMS_LENGTH                    =                                0;

  float RB_HEAT_TEMP_THRESH                  =                              -40;
  float RB_HEAT_TEMP_GAIN                    =                        0.1666;    // degrees c of temp to percent duty cycle
  float RB_HEAT_COMM_GAIN                    =                         1;    // hours of comm to percent duty cycle
  float RB_HEAT_CAP_GAIN                     =                                1;    // voltage drop to percent duty cycle
  float RB_HEAT_CAP_NOMINAL                     =                            4.5;    // voltage drop to percent duty cycle
  float RB_HEAT_MAX_DUTY                     =                              128;    // max duty cylce for heater in percent
  float RB_HEAT_DUTY                         =                                0;
  int32_t ACTIONS0 = {0};
  int32_t ACTIONS1 = {0};
  int32_t ACTIONS2 = {0};
  int32_t ACTIONS3 = {0};

  int32_t CUBA_NUMBER = 1973;

    float freq      =               20;        // control freqency
    float k         =               0.3;        // gain modifier
    float b_dldt    =            0.0002;        // balast dl/dt (kg/s)
    float v_dldt    =            0.001;        // valve dl/dt (kg/s)
    float rate_min  =          0.00001;        // min dl/dt rate threshold (kg/s)
    float rate_max  =            0.001;        // max dl/dt rate threshold (kg/s)
    float b_tmin    =               10;          // minimum ballast event time
    float v_tmin    =                5;          // minimum valve event time
    float h_cmd     =            13500;      // altidute comand

    float effort;             // Command effort from compensator
    float v_T;                // Interval time between vent events
    float b_T;                // Interval time between ballast events
    uint32_t v_ctr;                // valve interval counter
    uint32_t b_ctr;                // ballast interval counter
    uint32_t comp_ctr;
    int32_t action;               // action command

    float freq                          =               20;       // control freqency
    float k                             =                .6;       // gain modifier
    float b_dldt                        =           0.0002;       // balast dl/dt (kg/s)
    float v_dldt                        =            0.002;       // valve dl/dt (kg/s)
    float rate_min                      =          0.00001;       // min dl/dt rate threshold (kg/s)
    float rate_max                      =            0.001;       // max dl/dt rate threshold (kg/s)
    float b_tmin                        =               10;       // minimum ballast event time
    float v_tmin                        =                4;       // minimum valve event time
    float h_cmd                         =            14000;       // altidute comand
    float v_ss_error_thresh             =              750;
    float b_ss_error_thresh             =              750;
    float ascent_rate_thresh            =           100000;
    float kfuse                         =                0;
    float kfuse_v                       =              0.5;


    float effort;             // Command effort from compensator
    float v_T;                // Interval time between vent events
    float b_T;                // Interval time between ballast events
    uint32_t  v_ctr;                // valve interval counter
    uint32_t b_ctr;                // ballast interval counter
    uint32_t comp_ctr;
    int32_t action;               // action command
    float ascent_rate;        // filtered ascent rate
    float fused_ascent_rate;

    float freq                 =   20;        // control freqency
    float k_v                  =  .5*1e-3;      // velocity gain
    float k_h                  =  .5*1.5e-3;    // altitude gain
    float b_dldt               =   0.0002;    // balast dl/dt (kg/s)
    float v_dldt               =   0.0030;    // valve dl/dt (kg/s))
    float b_tmin               =   10;         // minimum ballast event time
    float v_tmin               =   5;         // minimum valve event time
    float h_cmd                =   13500;     // altidute comand
    float kfuse                =   7;
    float kfuse_val            =   0.5;
    float ss_error_thresh      =   750;
    float v_limit              =   0.5;
    float equil_h_thresh       =   10000;      //altitude where controller transitions to normal mode
    float launch_h_thresh      =   100;

    uint32_t comp_ctr      =   0;
    int32_t action                 =   0;               // action command
    float v                    =   0;
    float v1                   =   0;
    float v2                   =   0;
    float fused_v              =   0;
    float effort               =   0;
    float effort_sum           =   0;
    float v_cmd                =   0;
    uint8_t status              =   PRELAUNCH;


  uint32_t  ACTION_TIME_TOTALS0            =                               {0};
  uint32_t  ACTION_TIME_TOTALS1            =                               {0};
  uint32_t  ACTION_TIME_TOTALS2            =                               {0};
  uint32_t  ACTION_TIME_TOTALS3            =                               {0};
  uint32_t  ACTION_TIME_TOTALS4            =                               {0};
  uint32_t  ACTION_TIME_TOTALS5            =                               {0};
  uint32_t  ACTION_TIME_TOTALS6            =                               {0};
  uint32_t  ACTION_TIME_TOTALS7            =                               {0};
  float     OVERPRESSURE                     =                                 0;

