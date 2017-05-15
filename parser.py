#!/usr/bin/env python
# Stanford Student Space Initiative
# Balloons | VALBAL | April 2017
# Davy Ragland | dragland@stanford.edu

# File: parser.py
# --------------------------
# Server side script to parse incoming messages
# from RockBLOCK

#*******************************  SETUP  ***************************************
import math

#******************************  GLOBALS  **************************************
#binary is the string we get from RockBLOCK
binary = "00000000000000010101100000000000000000000100000000000000000000000001011100010110000110000110001100000000001010101111001111101111110000000000000000010101000000000000000000000000000000000000000000000100000110100000111010101000000010000000000001100000000100010011111100000000000000000000000011111111100100101010000000000001100001100010000000001000100000000000000101011111110010001111011000000110000110011111000001110000000000110000111111110101101101101101010100010100000010000010000100101000110011000000000000000001001100010000001000001000010010100011001100000000000000000001001111100000000000000000000000010000000010000000010000000110000000100011010000011100110001101000001111100000110100000111011000011010000011011110000101110001011000110100100000000000000000000000000000000000";
#Regex is what is pasted into "advanced parser"
regex = """lengthBits += compressVariable(data.TIME / 1000,                      0,    3000000, 20, lengthBits);
lengthBits += compressVariable(data.LAT_GPS,                         -90,   90,      21, lengthBits);
lengthBits += compressVariable(data.LONG_GPS,                        -180,  180,     22, lengthBits);
lengthBits += compressVariable(data.ALTITUDE,                        -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.ALTITUDE_GPS,                    -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.ASCENT_RATE,                     -10,   10,      11, lengthBits);
lengthBits += compressVariable(data.VALVE_INCENTIVE,                 -50,   10,      12, lengthBits);
lengthBits += compressVariable(data.BALLAST_INCENTIVE,               -50,   10,      12, lengthBits);
lengthBits += compressVariable(data.VALVE_STATE,                      0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BALLAST_STATE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.VALVE_QUEUE,                      0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.BALLAST_QUEUE,                    0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.NUM_VALVES,                       0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.NUM_BALLASTS,                     0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.NUM_VALVE_ATTEMPTS,               0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.NUM_BALLAST_ATTEMPTS,             0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.CUTDOWN_STATE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.PRESS,                            0,    500000,  19, lengthBits);
lengthBits += compressVariable(data.TEMP_IN,                         -50,   100,     9,  lengthBits);
lengthBits += compressVariable(data.JOULES,                           0,    1500000, 18, lengthBits);
lengthBits += compressVariable(data.VOLTAGE,                          0,    5,       9,  lengthBits);
lengthBits += compressVariable(data.CURRENT_AVG,                      0,    5000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_GPS_AVG,                  0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_RB_AVG,                   0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTORS_AVG,               0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD_AVG,              0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.TEMP_NECK,                       -100,  100,     9,  lengthBits);
lengthBits += compressVariable(data.TEMP_EXT,                        -100,  100,     9,  lengthBits);
lengthBits += compressVariable(data.SPEED_GPS,                       -100,  100,     9,  lengthBits);
lengthBits += compressVariable(data.HEADING_GPS,                     -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.NUM_SATS_GPS,                     0,    25,      4,  lengthBits);
lengthBits += compressVariable(data.LOOP_TIME,                        0,    10000,   10, lengthBits);
lengthBits += compressVariable(data.RB_SENT_COMMS,                    0,    8191,    13, lengthBits);
lengthBits += compressVariable(data.TEMP_SETPOINT,                   -20,   40,      6,  lengthBits);
lengthBits += compressVariable(data.MANUAL_MODE,                      0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.RB_SHOULD_USE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.GPS_SHOULD_USE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_SHOULD_USE,                0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.PAYLOAD_SHOULD_USE,               0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_STRONG_ENABLE,             0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_WEEK_ENABLE,               0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.GPS_GOOD_STATE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.SHOULD_REPORT,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.COMMS_INTERVAL,                 0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.GPS_INTERVAL,                   0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.PRESS_BASELINE,                 0,    500000,  19, lengthBits);
lengthBits += compressVariable(data.INCENTIVE_NOISE,                0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,            0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.RE_ARM_CONSTANT,                0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_ARM_ALT,               -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.VALVE_SETPOINT,                -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.VALVE_DURATION,                 0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.VALVE_FORCE_DURATION,           0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.VALVE_ALT_LAST,                -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.VALVE_VELOCITY_CONSTANT,        0,    5,       8,  lengthBits);
lengthBits += compressVariable(data.VALVE_ALTITUDE_DIFF_CONSTANT,   0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.VALVE_LAST_ACTION_CONSTANT,     0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_SETPOINT,              -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.BALLAST_DURATION,               0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_FORCE_DURATION,         0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_ALT_LAST,              -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.BALLAST_VELOCITY_CONSTANT,      0,    5,       8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_ALTITUDE_DIFF_CONSTANT, 0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_LAST_ACTION_CONSTANT,   0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.SHOULD_CUTDOWN,                 0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.SHOULD_LED,                     0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.SETUP_STATE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.DEBUG_STATE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.FORCE_VALVE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.FORCE_BALLAST,                  0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.REPORT_MODE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_1_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_2_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_3_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_4_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_1_REJECTIONS,               0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BMP_2_REJECTIONS,               0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BMP_3_REJECTIONS,               0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BMP_4_REJECTIONS,               0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.RAW_TEMP_1,                    -50,   100,     9,  lengthBits);
lengthBits += compressVariable(data.RAW_TEMP_2,                    -50,   100,     9,  lengthBits);
lengthBits += compressVariable(data.RAW_TEMP_3,                    -50,   100,     9,  lengthBits);
lengthBits += compressVariable(data.RAW_TEMP_4,                    -50,   100,     9,  lengthBits);
lengthBits += compressVariable(data.RAW_PRESSURE_1,                 0,    1000000, 19, lengthBits);
lengthBits += compressVariable(data.RAW_PRESSURE_2,                 0,    1000000, 19, lengthBits);
lengthBits += compressVariable(data.RAW_PRESSURE_3,                 0,    1000000, 19, lengthBits);
lengthBits += compressVariable(data.RAW_PRESSURE_4,                 0,    1000000, 19, lengthBits);
lengthBits += compressVariable(data.CURRENT,                        0,    5000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_GPS,                    0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_RB,                     0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTORS,                 0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD,                0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.ALTITUDE_LAST,                 -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.GPS_LAST,                       0,    500000,  10, lengthBits);
lengthBits += compressVariable(data.COMMS_LAST,                     0,    500000,  10, lengthBits);
lengthBits += compressVariable(data.DATAFILE_LAST,                  0,    500000,  10, lengthBits);
lengthBits += compressVariable(data.COMMS_LENGTH,                   0,    200,     8,  lengthBits);"""

names = []
mins  = []
maxs  = []
bits  = []

#******************************  HELPERS  *************************************
def setupREGEX():
    for line in regex.split('\n'):
        csv = line.split(",")
        names.append(csv[0].split("data.")[1])
        mins.append(int(csv[1].replace(" ", "")))
        maxs.append(int(csv[2].replace(" ", "")))
        bits.append(int(csv[3].replace(" ", "")))

def parseMessage(message):
    curr = 0
    for i in range(len(names)):
        num = message[curr:(curr + bits[i])]
        curr = curr + bits[i]
        adc = int(num, 2)
        value = mins[i] + adc * ((maxs[i] - mins[i]) / (math.pow(2, bits[i]) - 1))
        print(names[i] + ":" + str(value))

#********************************  MAIN  ***************************************
setupREGEX()
parseMessage(binary)
