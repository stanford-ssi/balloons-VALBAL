#include <Arduino.h>

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   POT_CS                                =                4;
static const uint8_t   CURRENT_MONITOR_CS                    =               40;
static const uint8_t   BMP_CS_ONE                            =               57;
static const uint8_t   BMP_CS_TWO                            =               55;
static const uint8_t   BMP_CS_THREE                          =               53;
static const uint8_t   BMP_CS_FOUR                           =                5;
static const uint8_t   BMPX_CS_ONE                           =               54;
static const uint8_t   BMPX_CS_TWO                           =               34;
static const uint8_t   VALVE_FORWARD                         =               46;
static const uint8_t   VALVE_REVERSE                         =               43;
static const uint8_t   VALVE_ENCA                       	 =               45;
static const uint8_t   VALVE_ENCB                       	 =               15;
static const uint8_t   VALVE_ENCPWR	                    	 =               44;
static const uint8_t   BALLAST_FORWARD                       =               41;
static const uint8_t   BALLAST_REVERSE                       =               42;
static const uint8_t   BALLAST_ENCA                       	 =               29;
static const uint8_t   BALLAST_ENCB                       	 =               30;
static const uint8_t   BALLAST_ENCPWR                    	 =               40;
static const uint8_t   GPS_GATE                              =               36;
static const uint8_t   RB_GATE                               =               28;
static const uint8_t   RB_SLEEP                              =               39;
static const uint8_t   PAYLOAD_GATE                          =               35;
static const uint8_t   PAYLOAD_GPIO_1                        =              A22;
static const uint8_t   PAYLOAD_GPIO_2                        =               52;
static const uint8_t   PAYLOAD_DAC                           =              A21;
static const uint8_t   PAYLOAD_DAC_2                         =              A22;
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
static const uint8_t   CUTDOWN_FWD                        =               6;
static const uint8_t   CUTDOWN_REV                        =               2;

static const uint8_t   CURRENT_SENSOR_CS                     =                1;

static const uint8_t ISENSE_RB = 38;
static const uint8_t ISENSE_MOT = 37;
static const uint8_t ISENSE_SD = 17;
static const uint8_t ISENSE_MAIN = 16;
static const uint8_t ISENSE_PLD = A10;
static const uint8_t ISENSE_GPS = A11;
static const uint8_t VSENSE_MAIN = 23;
static const uint8_t VSENSE_CAP = 19;

const int SENSOR_IRB = 0;
const int SENSOR_IMOT = 1;
const int SENSOR_IMAIN = 2;
const int SENSOR_IPLD = 3;
const int SENSOR_ISD = 4;
const int SENSOR_IGPS = 5;
const int SENSOR_VMAIN = 6;
const int SENSOR_VCAP = 7;

static const int OP_PIN = A12;
static const int VR_PIN = A21;
