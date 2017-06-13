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
regex = """lengthBits += compressVariable(data.TIME / 1000,                           0,    3000000, 20, lengthBits); // time
lengthBits += compressVariable(data.LAT_GPS,                              -90,   90,      21, lengthBits); // latitude
lengthBits += compressVariable(data.LONG_GPS,                             -180,  180,     22, lengthBits); // longitude
lengthBits += compressVariable(data.ALTITUDE_BAROMETER,                   -2000, 40000,   16, lengthBits); // altitude_barometer
lengthBits += compressVariable(data.ALTITUDE_GPS,                         -2000, 40000,   14, lengthBits);
lengthBits += compressVariable(data.ASCENT_RATE,                          -10,   10,      11, lengthBits);
lengthBits += compressVariable(data.VALVE_INCENTIVE,                      -50,   10,      12, lengthBits);
lengthBits += compressVariable(data.BALLAST_INCENTIVE,                    -50,   10,      12, lengthBits);
lengthBits += compressVariable(data.VALVE_STATE,                           0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BALLAST_STATE,                         0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.VALVE_QUEUE / 1000,                    0,    1023,    10, lengthBits);
lengthBits += compressVariable(data.BALLAST_QUEUE / 1000,                  0,    1023,    10, lengthBits);
lengthBits += compressVariable(data.VALVE_TIME_TOTAL / 1000,               0,    16383,   13, lengthBits); // valve time total
lengthBits += compressVariable(data.BALLAST_TIME_TOTAL / 1000,             0,    16383,   13, lengthBits); // ballast time total
lengthBits += compressVariable(data.VALVE_NUM_ACTIONS,                     0,    63,      6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_NUM_ACTIONS,                   0,    63,      6,  lengthBits);
lengthBits += compressVariable(data.VALVE_NUM_ATTEMPTS,                    0,    63,      6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_NUM_ATTEMPTS,                  0,    63,      6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_NUM_OVERCURRENTS,              0,    63,      6,  lengthBits);
lengthBits += compressVariable(data.CUTDOWN_STATE,                         0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.TEMP_INT,                             -85,   65,      9,  lengthBits);
lengthBits += compressVariable(data.JOULES_TOTAL,                          0,    1572863, 18, lengthBits);
lengthBits += compressVariable(data.VOLTAGE_PRIMARY,                       0,    6,       9,  lengthBits);
lengthBits += compressVariable(data.VOLTAGE_5V,                            4,    6,       7,  lengthBits);
lengthBits += compressVariable(data.CURRENT_TOTAL_AVG,                     0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.CURRENT_TOTAL_MIN,                     0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.CURRENT_TOTAL_MAX,                     0,    4095,    12, lengthBits);
lengthBits += compressVariable(data.CURRENT_RB_AVG,                        0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_RB_MAX,                        0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTOR_VALVE_AVG,               0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTOR_VALVE_MAX,               0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTOR_BALLAST_AVG,             0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_MOTOR_BALLAST_MAX,             0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD_AVG,                   0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.CURRENT_PAYLOAD_MAX,                   0,    1023,    8,  lengthBits);
lengthBits += compressVariable(data.TEMP_EXT,                             -100,  30,      8,  lengthBits);
lengthBits += compressVariable(data.LOOP_TIME_MAX,                         0,    10239,   10, lengthBits);
lengthBits += compressVariable(data.RB_SENT_COMMS,                         0,    8191,    13, lengthBits);
lengthBits += compressVariable(data.RB_SLEEP_FAILS,                        0,    8191,    13, lengthBits);
lengthBits += compressVariable(data.MANUAL_MODE,                           0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.REPORT_MODE,                           0,    2,       2,  lengthBits);
lengthBits += compressVariable(data.SHOULD_REPORT,                         0,    1,       1,  lengthBits);
if (data.SHOULD_REPORT || data.REPORT_MODE != 0) {
lengthBits += compressVariable(data.POWER_STATE_LED,                     0,    1,       1,  lengthBits); // LED Power state
lengthBits += compressVariable(data.POWER_STATE_RB,                      0,    1,       1,  lengthBits); // RB Power State
lengthBits += compressVariable(data.POWER_STATE_GPS,                     0,    1,       1,  lengthBits); // GPS Power State
lengthBits += compressVariable(data.POWER_STATE_HEATER,                  0,    1,       1,  lengthBits); // Heater Power State
lengthBits += compressVariable(data.POWER_STATE_PAYLOAD,                 0,    1,       1,  lengthBits); // Payload Power State
lengthBits += compressVariable(data.HEATER_STRONG_ENABLE,                0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.HEATER_WEEK_ENABLE,                  0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.NUM_SATS_GPS,                        0,    15,      3,  lengthBits);
lengthBits += compressVariable(data.INCENTIVE_NOISE,                     0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.RE_ARM_CONSTANT,                     0,    4,       8,  lengthBits);
lengthBits += compressVariable(data.VALVE_ALT_LAST,                     -2000, 50000,   11, lengthBits); // Altitude During Last Venting Event
lengthBits += compressVariable(data.BALLAST_ALT_LAST,                   -2000, 50000,   11, lengthBits); // Altitude During Last Ballast Event
lengthBits += compressVariable(data.DEBUG_STATE,                         0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.FORCE_VALVE,                         0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.FORCE_BALLAST,                       0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_1_ENABLE,                        0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_2_ENABLE,                        0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_3_ENABLE,                        0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.BMP_4_ENABLE,                        0,    1,       1,  lengthBits);
lengthBits += compressVariable(log2(data.BMP_1_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_1_logrejections
lengthBits += compressVariable(log2(data.BMP_2_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_2_logrejections
lengthBits += compressVariable(log2(data.BMP_3_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_3_logrejections
lengthBits += compressVariable(log2(data.BMP_4_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_4_logrejections
lengthBits += compressVariable(data.BLACK_BODY_TEMP,                    -100,  30,      8,  lengthBits);
lengthBits += compressVariable(data.JOULES_HEATER,                       0,    819199,  13, lengthBits);
}
if (data.SHOULD_REPORT || data.REPORT_MODE == 2) {
lengthBits += compressVariable(data.TEMP_SETPOINT,                      -70,   40,      6,  lengthBits); // Payload temperature setpoint
lengthBits += compressVariable(data.RB_INTERVAL / 1000,                  0,    1023,    10, lengthBits); // RB communication interval
lengthBits += compressVariable(data.GPS_INTERVAL / 1000,                 0,    1023,    10, lengthBits); // GPS communication interval
lengthBits += compressVariable(data.RB_SHOULD_SLEEP,                     0,    1,       1,  lengthBits);
lengthBits += compressVariable(data.PRESS_BASELINE,                      0,    131071,  17, lengthBits); // Pressure baseline
lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,                 0,    4,       3,  lengthBits);
lengthBits += compressVariable(data.BALLAST_ARM_ALT,                    -2000, 40000,   16, lengthBits); // Ballast Arming Altitude
lengthBits += compressVariable(data.BALLAST_REVERSE_INTERVAL / 1000,     0,    1599,    4,  lengthBits); // Ballast reverse interval
lengthBits += compressVariable(data.VALVE_LEAK_INTERVAL / 1000,          0,    1599,    4,  lengthBits);
lengthBits += compressVariable(data.BALLAST_STALL_CURRENT,               0,    511,     4,  lengthBits);
lengthBits += compressVariable(data.VALVE_OPENING_DURATION / 1000,       0,    10,      5,  lengthBits);
lengthBits += compressVariable(data.VALVE_CLOSING_DURATION / 1000,       0,    10,      5,  lengthBits);
lengthBits += compressVariable(data.VALVE_SETPOINT,                     -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.VALVE_VENT_DURATION / 1000,          0,    1023,    6,  lengthBits);
lengthBits += compressVariable(data.VALVE_FORCE_DURATION / 1000,         0,    1023,    6,  lengthBits);
lengthBits += compressVariable(data.VALVE_VELOCITY_CONSTANT,             0,    5,       8,  lengthBits); // Valve Speed Constant
lengthBits += compressVariable(1.0 / data.VALVE_ALTITUDE_DIFF_CONSTANT,  0,    4095,    8,  lengthBits); // Valve Altitude Difference Constant
lengthBits += compressVariable(1.0 / data.VALVE_LAST_ACTION_CONSTANT,    0,    4095,    8,  lengthBits); // Valve last action constant
lengthBits += compressVariable(data.BALLAST_SETPOINT,                   -2000, 50000,   11, lengthBits);
lengthBits += compressVariable(data.BALLAST_DROP_DURATION / 1000,        0,    1023,    6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_FORCE_DURATION / 1000,       0,    1023,    6,  lengthBits);
lengthBits += compressVariable(data.BALLAST_VELOCITY_CONSTANT,           0,    5,       8,  lengthBits); // Ballast Speed Constant
lengthBits += compressVariable(1.0 / data.BALLAST_ALTITUDE_DIFF_CONSTANT,0,    4095,    8,  lengthBits); // Ballast Altitude Difference Constant
lengthBits += compressVariable(1.0 / data.BALLAST_LAST_ACTION_CONSTANT,  0,    4095,    8,  lengthBits); // Ballast last action constant"""

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
