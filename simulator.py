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
            print line
            ser.write(line)
            ser.write('\n')
            time.sleep(0.05)

#********************************  MAIN  ***************************************
parseArgs(sys.argv[1:])
feedData()
