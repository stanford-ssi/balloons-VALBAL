#!/usr/bin/env python
# Stanford Student Space Initiative
# Balloons | VALBAL | May 2017
# Davy Ragland | dragland@stanford.edu

# File: parser.py
# --------------------------
# Server side script to parse incoming messages
# from RockBLOCK

#*******************************  SETUP  ***************************************
import math

#******************************  GLOBALS  **************************************
#binary is the string we get from RockBLOCK
binary = "0000000000010100010010110101001110101011000101001000111101101100000110010001010000011000110010101111111111101001000011000000000000000000000000000000000000000000000000000000000000000001010011100000011101011100011011101000000101011100000000000000000011110010011000100111110000010110010001100100110001001110001011100110011111111000000011100000000001010000101000000011100000011101111101000000000000000000000000001111111100001001111000000000000100111100000000000000001000000001111011000000110000110011111000001110011000001011011011011011001100011111111000110100001010001010000001000001001100110000000000000000010011000100000010000010011001100000000000000000000000000000";
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
lengthBits += compressVariable(data.NUM_VALVES,                       0,    50,      6,  lengthBits);
lengthBits += compressVariable(data.NUM_BALLASTS,                     0,    50,      6,  lengthBits);
lengthBits += compressVariable(data.NUM_VALVE_ATTEMPTS,               0,    50,      6,  lengthBits);
lengthBits += compressVariable(data.NUM_BALLAST_ATTEMPTS,             0,    50,      6,  lengthBits);
lengthBits += compressVariable(data.NUM_BALLAST_OVER_CURRENTS,        0,    50,      6,  lengthBits);
lengthBits += compressVariable(data.CUTDOWN_STATE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.TEMP_IN,                         -70,   80,      9,  lengthBits);
lengthBits += compressVariable(data.JOULES,                           0,    1500000, 18, lengthBits);
lengthBits += compressVariable(data.VOLTAGE,                          0,    5,       9,  lengthBits);
lengthBits += compressVariable(data.CURRENT_AVG,                      0,    4000,    12, lengthBits);
lengthBits += compressVariable(data.CURRENT_MIN,                      0,    4000,    12, lengthBits);
lengthBits += compressVariable(data.CURRENT_MAX,                      0,    4000,    12, lengthBits);
lengthBits += compressVariable(data.CURRENT_GPS_AVG,                  0,    200,     8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_GPS_MAX,                  0,    200,     8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_RB_AVG,                   0,    1000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_RB_MAX,                   0,    1000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTORS_AVG,               0,    1000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTORS_MAX,               0,    1000,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD_AVG,              0,    500,     8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD_MAX,              0,    500,     8,  lengthBits);
lengthBits += compressVariable(data.TEMP_EXT,                        -100,  30,      6,  lengthBits);
lengthBits += compressVariable(data.LOOP_TIME,                        0,    10000,   10, lengthBits);
lengthBits += compressVariable(data.RB_SENT_COMMS,                    0,    8191,    13, lengthBits);
lengthBits += compressVariable(data.EULER_X_AVG,                      0,    360,     8,  lengthBits);
lengthBits += compressVariable(data.EULER_Y_AVG,                     -180,  180,     8,  lengthBits);
lengthBits += compressVariable(data.EULER_Z_AVG,                     -90,   90,      7,  lengthBits);
lengthBits += compressVariable(data.MANUAL_MODE,                      0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.REPORT_MODE,                      0,    2,       2,  lengthBits);
lengthBits += compressVariable(data.SHOULD_REPORT,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.RB_SHOULD_USE,                  0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.GPS_SHOULD_USE,                 0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_SHOULD_USE,              0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.PAYLOAD_SHOULD_USE,             0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_STRONG_ENABLE,           0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_WEEK_ENABLE,             0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.GPS_GOOD_STATE,                 0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.SPEED_GPS,                      0,    150,     7,  lengthBits);
lengthBits += compressVariable(data.HEADING_GPS,                    0,    360,     8,  lengthBits);
lengthBits += compressVariable(data.NUM_SATS_GPS,                   0,    25,      3,  lengthBits);
lengthBits += compressVariable(data.INCENTIVE_NOISE,                0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.RE_ARM_CONSTANT,                0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.VALVE_ALT_LAST,                -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.BALLAST_ALT_LAST,              -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.SHOULD_LED,                     0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.DEBUG_STATE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.FORCE_VALVE,                    0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.FORCE_BALLAST,                  0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_1_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_2_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_3_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_4_ENABLE,                   0,    1,       1,  lengthBits);
lengthBits += compressVariable(log2(data.BMP_1_REJECTIONS + 1),     0,    6,       4,  lengthBits);
lengthBits += compressVariable(log2(data.BMP_2_REJECTIONS + 1),     0,    6,       4,  lengthBits);
lengthBits += compressVariable(log2(data.BMP_3_REJECTIONS + 1),     0,    6,       4,  lengthBits);
lengthBits += compressVariable(log2(data.BMP_4_REJECTIONS + 1),     0,    6,       4,  lengthBits);
lengthBits += compressVariable(data.TEMP_SETPOINT,                 -40,   40,      6,  lengthBits);
lengthBits += compressVariable(data.COMMS_INTERVAL,                 0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.GPS_INTERVAL,                   0,    1000000, 10, lengthBits);
lengthBits += compressVariable(data.PRESS_BASELINE,                 0,    500000,  19, lengthBits);
lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,            0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_ARM_ALT,               -2000, 40000,   16, lengthBits);
lengthBits += compressVariable(data.BALLAST_REVERSE_TIMEOUT,        0,    1000000, 4,  lengthBits);
lengthBits += compressVariable(data.BALLAST_STALL_CURRENT,          0,    500,     4,  lengthBits);
lengthBits += compressVariable(data.VALVE_MOTOR_SPEED,              0,    255,     4,  lengthBits);
lengthBits += compressVariable(data.BALLAST_MOTOR_SPEED,            0,    255,     4,  lengthBits);
lengthBits += compressVariable(data.VALVE_OPENING_TIMEOUT,          0,    10000,   5,  lengthBits);
lengthBits += compressVariable(data.VALVE_CLOSING_TIMEOUT,          0,    10000,   5,  lengthBits);
lengthBits += compressVariable(data.VALVE_SETPOINT,                -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.VALVE_DURATION,                 0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.VALVE_FORCE_DURATION,           0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.VALVE_VELOCITY_CONSTANT,        0,    5,       8,  lengthBits);
lengthBits += compressVariable(data.VALVE_ALTITUDE_DIFF_CONSTANT,   0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.VALVE_LAST_ACTION_CONSTANT,     0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_SETPOINT,              -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.BALLAST_DURATION,               0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_FORCE_DURATION,         0,    1000000, 6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_VELOCITY_CONSTANT,      0,    5,       8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_ALTITUDE_DIFF_CONSTANT, 0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.BALLAST_LAST_ACTION_CONSTANT,   0,    4000,    8,  lengthBits);
lengthBits += compressVariable(data.EULER_HISTORY,                  0,    30,      5,  lengthBits);"""

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
