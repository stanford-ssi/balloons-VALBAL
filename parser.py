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
binary = "0000000000010100010010110101001110100010100101001001000010010000000101110010000000011000100100101011101001100011111001111111111111000000000000000000011100000000000000000000000000000000000100000010111110011010000011100101011101011000000001001110000000010010001101010000000000000000000000001111111111000000000000110000110001010100000100010000000000111000111101100000011000101011111101001100111110000011100000000001100001111111101011011011011010100110001000000100001001011001100110000000000000000010010011100000010000100101100110011000000000000000001010011111100000000000000000000000001110101101110101001110101101110101100011010000011101010001101000001110110000110100000111001000011010000011011010000101110010010111111111111111111111111111111111111110011110100000";
#Regex is what is pasted into "advanced parser"
regex = """  lengthBits += compressVariable(data.TIME / 1000,                      0,    3000000, 20, lengthBits);
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
lengthBits += compressVariable(data.TEMP,                            -50,   100,     9,  lengthBits);
lengthBits += compressVariable(data.JOULES,                           0,    1500000, 18, lengthBits);
lengthBits += compressVariable(data.VOLTAGE,                          0,    5,       9,  lengthBits);
lengthBits += compressVariable(data.CURRENT,                          0,    5000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_GPS,                      0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_RB,                       0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTORS,                   0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD,                  0,    3000,    6,  lengthBits);
lengthBits += compressVariable(data.TEMP_NECK,                       -100,  100,     9,  lengthBits);
lengthBits += compressVariable(data.SPEED_GPS,                       -100,  100,     9,  lengthBits);
lengthBits += compressVariable(data.HEADING_GPS,                     -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.NUM_SATS_GPS,                     0,    25,      4,  lengthBits);
lengthBits += compressVariable(data.LOOP_TIME,                        0,    10000,   10, lengthBits);
lengthBits += compressVariable(data.RB_SENT_COMMS,                    0,    8191,    13, lengthBits);
lengthBits += compressVariable(data.COMMS_INTERVAL,                   0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.GPS_INTERVAL,                     0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.TEMP_SETPOINT,                   -20,   40,      6,  lengthBits);
lengthBits += compressVariable(data.MANUAL_MODE,                      0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.RB_SHOULD_USE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.GPS_SHOULD_USE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_SHOULD_USE,                0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_STRONG_ENABLE,             0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_WEEK_ENABLE,               0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.GPS_GOOD_STATE,                   0,    1,       1,  lengthBits);
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
lengthBits += compressVariable(data.SHOULD_REPORT,                  0,    1,       1,  lengthBits);
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
lengthBits += compressVariable(data.ALTITUDE_LAST,                 -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.GPS_LAST,                       0,    500000,  19, lengthBits);
lengthBits += compressVariable(data.COMMS_LAST,                     0,    500000,  19, lengthBits);
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
