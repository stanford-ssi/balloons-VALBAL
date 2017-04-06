#!/usr/bin/env python
# Stanford Student Space Initiative
# Balloons | VALBAL | April 2017
# Davy Ragland | dragland@stanford.edu

# File: simulator.py
# --------------------------
# Server side script to feed simuled data to VALBAL
# over Serial for Hardware in the Loop testing.

#*******************************  SETUP  ***************************************
import sys
import getopt
import time
import serial

#******************************  GLOBALS  **************************************
filename = ""
timeStamp = 40000
ser = serial.Serial("/dev/ttyACM0", 115200)

#******************************  HELPERS  *************************************
def usageError():
    print("Usage: simulator.py -l [LOG]")
    sys.exit(2)

def parseArgs(argv):
    global filename
    try:
        opts, args = getopt.getopt(argv,"l:")
    except getopt.GetoptError:
        usageError()
    if len(argv) != 2:
        usageError()
    for opt, arg in opts:
        if opt == '-l':
            filename = arg
def feedData():
    print("Loading data...")
    global timeStamp
    with open(filename) as f:
        next(f)
        next(f)
        for line_terminated in f:
            line = line_terminated.rstrip('\n')
            csv = line.split(',')
            time.sleep((long(csv[0]) - timeStamp) / 1000.0);
            timeStamp = long(csv[0])
            print line
            ser.write(line)
            ser.write('\n')

#********************************  MAIN  ***************************************
parseArgs(sys.argv[1:])
feedData()

# #include "Arduino.h"
#
# /***********************************  BOOT  ***********************************/
# static const uint16_t UART_BUFFER_SIZE = 1024;
# char buffer[UART_BUFFER_SIZE] = {0};
#
# /********************************  FUNCTIONS  *********************************/
# size_t getLine() {
#   for (size_t i = 0; i < UART_BUFFER_SIZE; i++) buffer[i] = 0;
#   size_t i = 0;
#   while(Serial.available()) {
#     char c = Serial.read();
#     if(c == '\n') {
#       return i;
#     }
#     buffer[i] = c;
#     i++;
#   }
# }
# /***********************************  MAIN  ***********************************/
# int main() {
#   Serial.begin(115200);
#   while(true) {
#     size_t len = getLine();
#     if (len != 0) {
#       for (size_t i = 0; i < len; i++) Serial.print(buffer[i]);
#       Serial.print('\n');
#     }
#   }
# }
