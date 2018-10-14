#include <Arduino.h>

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   SD_CS                                 =               23;
static const uint8_t   CURRENT_MONITOR_CS                    =               40;
static const uint8_t   BMP_CS_ONE                            =               57;
static const uint8_t   BMP_CS_TWO                            =               55;
static const uint8_t   BMP_CS_THREE                          =               53;
static const uint8_t   BMP_CS_FOUR                           =                5;
static const uint8_t   BMPX_CS_ONE                           =               56;
static const uint8_t   BMPX_CS_TWO                           =               51;
static const uint8_t   VALVE_FORWARD                         =               46;
static const uint8_t   VALVE_REVERSE                         =               43;
static const uint8_t   BALLAST_FORWARD                       =               41;
static const uint8_t   BALLAST_REVERSE                       =               42;
static const uint8_t   GPS_GATE                              =               36;
static const uint8_t   RB_GATE                               =               28;
static const uint8_t   RB_SLEEP                              =               39;
static const uint8_t   PAYLOAD_GATE                          =               35;
static const uint8_t   PAYLOAD_GPIO_1                        =              A22;
static const uint8_t   PAYLOAD_GPIO_2                        =               52;
static const uint8_t   PAYLOAD_DAC                           =              A21;
static const uint8_t   PAYLOAD_DAC_2                         =              A22;
static const uint8_t   BATT_VOLTAGE                          =               23;
static const uint8_t   SUPER_CAP_VOLTAGE                     =               19;
static const uint8_t   SUPER_CAP_ENABLE                      =               50;
static const uint8_t   FIVE_VOLT_ENABLE                      =               49;
static const uint8_t   TOTAL_CURRENT                         =                1;
static const uint8_t   RB_CURRENT                            =                2;
static const uint8_t   MOTORS_CURRENT                        =                3;
static const uint8_t   PAYLOAD_CURRENT                       =                4;
static const uint8_t   EXT_TEMP_SENSOR                       =               36;
static const uint8_t   CUTDOWN_POWER                         =               27;
static const uint8_t   CUTDOWN_SIGNAL                        =               27;
static const uint8_t   CUTDOWN_FWD                           =                6;
static const uint8_t   CUTDOWN_REV                           =                2;
static const uint8_t   LED_PIN                               =               25;
  static const uint8_t NUM_LEDS                              =                1;
static const uint8_t   CURRENT_SENSOR_CS                     =                1;


static const int OP_PIN = A12;
static const int VR_PIN = A21;
