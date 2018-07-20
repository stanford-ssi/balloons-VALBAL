#include <Arduino.h>

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   SD_CS                                 =               23;
static const uint8_t   CURRENT_MONITOR_CS                    =               40;
static const uint8_t   BMP_CS_ONE                            =               55;
static const uint8_t   BMP_CS_TWO                            =               53;
static const uint8_t   BMP_CS_THREE                          =               51;
static const uint8_t   BMP_CS_FOUR                           =               54;
static const uint8_t   VALVE_FORWARD                         =               16;
static const uint8_t   VALVE_REVERSE                         =                4;
static const uint8_t   BALLAST_FORWARD                       =               42;
static const uint8_t   BALLAST_REVERSE                       =                3;
static const uint8_t   GPS_GATE                              =               26;
static const uint8_t   RB_GATE                               =               46;
static const uint8_t   RB_SLEEP                              =               14;
static const uint8_t   PAYLOAD_GATE                          =               57;
static const uint8_t   PAYLOAD_GPIO_1                        =               29;
static const uint8_t   PAYLOAD_GPIO_2                        =               30;
static const uint8_t   PAYLOAD_DAC                           =              A14;
static const uint8_t   BATT_VOLTAGE                          =               A3;
static const uint8_t   SUPER_CAP_VOLTAGE                     =              A15;
static const uint8_t   SUPER_CAP_ENABLE                      =               49;
static const uint8_t   FIVE_VOLT_ENABLE                      =               56;
static const uint8_t   TOTAL_CURRENT                         =                1;
static const uint8_t   RB_CURRENT                            =                2;
static const uint8_t   MOTORS_CURRENT                        =                3;
static const uint8_t   PAYLOAD_CURRENT                       =                4;
static const uint8_t   EXT_TEMP_SENSOR                       =               36;
static const uint8_t   CUTDOWN_POWER                         =               27;
static const uint8_t   CUTDOWN_SIGNAL                        =               28;

static const uint8_t   CURRENT_SENSOR_CS                     =                1;


static const int OP_PIN = A12;
static const int VR_PIN = A21;
