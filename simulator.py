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
VALBAL = "/dev/ttyACM0"
timeStamp = 12000

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
            # ser.write(struct.pack(line)

#********************************  MAIN  ***************************************
parseArgs(sys.argv[1:])
# ser = serial.Serial(VALBAL, 115200)
feedData()


# void getLine() {
#   while(true) {
#     if(Serial.available()) {
#       char c = Serial.read();
#       if(c == '\n') {
#         Serial.print('\n');
#         return;
#       }
#       Serial.print(c);
#     }
#   }
# }
#
# int main() {
#   Serial.begin(115200);
#   while(true) {
#     getLine();
#   }
# }
