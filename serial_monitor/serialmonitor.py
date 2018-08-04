import serial
import struct
import time as py_time
import numpy as np
from sys import argv, exit
import argparse
import re
import pty
import os
from shitl2sm import SerialMonitorSocket

from flask import Flask, render_template, Response, stream_with_context
import queue
import _thread

# constants - need to have values specified by command line arguments
TEENSY_ADR = '/dev/tty.usbmodem144121'#'/dev/ttyACM0'
srcname = "../src/SerialMonitor.cpp"
log_file_path = "sample_shitl.txt"
log_file_write_path = "sample_shitl_copy.txt"
FSTART = b'\xaa'
EXTRA_START = '~'

# flags - false by default
WILL_LOG = False
READ_LOG = False
DEBUG = False

# communication between serial reading and server part
serial_queue = queue.Queue()

# file for logging
log_file = None


# Set up the parser and parse those arguments
parser = argparse.ArgumentParser(description="A valbal serial monitor")
parser.add_argument('--read', '-r', help="specifiy a file to read saved serial data from", type=str)
parser.add_argument('--teensy', '-t', help="specify the path to the teensy to read from", type=str)
parser.add_argument('--write', '-w', help="specify a file where you want to save all of the serial data that was read. Used for creating sample serial data files", type=str)
parser.add_argument('--debug', '-d', help="debug flag", action="store_true")
parser.add_argument('--shitl', '-s', help="shitl flag. Will be expecting data sent over a socket from shitl.py", action="store_true")
args = parser.parse_args()

SHITL = args.shitl

if ((args.read is None and args.teensy is None) or (args.read is not None and args.teensy is not None)) and not SHITL:
    print(">>> ERROR: You must specify a file to read serial data from (-r) [exclusive] OR a path to the teensy (something like '/dev/tty.usbmodem144121') ")
    exit(1)

if (args.read is not None):
    READ_LOG = True
    log_file_path = args.read

if (args.write is not None):
    WILL_LOG = True
    log_file_path = args.write

if WILL_LOG and READ_LOG:
    print(">>> ERROR: CANNOT READ AND WRITE TO LOG FILE AT THE SAME TIME")
    exit(1)

if (args.teensy is not None):
    READ_LOG = False
    TEENSY_ADR = args.teensy


DEBUG = args.debug


# setup log file stuff
if WILL_LOG: log_file = open(log_file_path, 'ab')
if READ_LOG: log_file = open(log_file_path, 'rb')



############################################   SERIAL PART    #################################################


# more organized code (with functions!)


# returns either the serial object or the file with the valbal output depending on what was specified by the user
def get_teensy():
    if READ_LOG:
        return log_file
    elif SHITL:
        return SerialMonitorSocket()
    else:
        return serial.Serial(TEENSY_ADR)

# parses SerialMonitor.cpp to extract the names and number of variables we care about (from John's code in shitl.py)
def get_vars(srcname):
    names = []
    num_report = 0
    f = open(srcname, 'r')
    lines = f.readlines()
    for i,line in enumerate(lines):
        if re.search('diddlybop', line): # this is why that comment matters
            num_report = int(re.findall(r"\[(.*?)\]",lines[i+1])[0])
            names = [re.findall(r"\= (.*?)\;",k)[0] for k in lines[i+2:i+2+num_report]]
            if DEBUG: print(names)
            break
    f.close()
    names.insert(0,'time')
    return (names, num_report)

# add message to queue to be picked up by server
def send_message(message):
    serial_queue.put(message)

# main serial reading function adapted from John's code in shitl.py
def run_serial(teensy, names, num_report):
    # Await teensy response to make sure we can talk
    # But only do this if it's the first time reading in serial
    buf = ""
    using_extra = False
    while (1):
        read = teensy.read(1)

        if WILL_LOG: log_file.write(read)

        # teensy will send FSTART when it's ready
        if read == FSTART:
            if DEBUG: print('>>> VALBAL found ready to monitor serial')

            #if not READ_LOG: teensy.write(FSTART) # if the serial monitor is re-run, it will send an FSTART regardless of if the Teensy is expecting it, 
                                 # so make sure FSTARTs are dealt with on the teensy side in an appropriate manner
            #print("free", read)
            break
        else:
            pass
            #print("Got it", read)


    # Read teensy data
    if DEBUG: print('reading data')
    while(1):
        if READ_LOG: py_time.sleep(0.001)
        
        py_time.sleep(0.001)
        read = teensy.read(1)

        if WILL_LOG: log_file.write(read)
        #print("In main loop: ", read)
        # now and FSTART signals that the variables we want to display are coming next
        if read == FSTART:
            if DEBUG: print('>>> Ready to read VALBAL data')

            # read the time first
            request = teensy.read(4) # takes number of bytes read
            time = struct.unpack('I', request)[0]/1000 # the 'I' means treat the bytes we read as an integer representing # of millis so convert to seconds

            if DEBUG: print('>>> Request Time:',time)
            if WILL_LOG: log_file.write(request)
            
            # read the rest of the data: (num_report floats)
            data = teensy.read(4*num_report)

            status = list(struct.unpack('f'*num_report, data)) # there is one 'f' for each float that has to be read
            
            status.insert(0, time) # we want the time to be at the top

            # if there's text we haven't sent to the server yet, send it and reset the buffer
            if (len(buf) > 0):
                if DEBUG: print(buf)
                send_message(buf)
                buf = ""
            
            if DEBUG: print(">>> VB Status:")

            # add the labels to the floats and semi-colon separate the variables
            for i in range(num_report):
                buf += "\t%s:\t%f;"%(names[i], status[i])
                if DEBUG: print("\t%s:\t%f"%(names[i], status[i]))
            if WILL_LOG: log_file.write(data)

            buf = buf[:-1] # shave off the last semicolon
            send_message(buf)
            buf = ""

        # we're reading extra info (not variables we care about). This will be displayed on the bottom half of the serial monitor screen
        else:
            try:
                if (len(buf) == 0):
                    # mark the beginning of the extra information with a special symbol to tell the client that it's extra
                    buf = EXTRA_START

                next_chr = read.decode("utf-8")

                # push whole lines to the client so it looks decent
                if (next_chr == "\n"):
                    send_message(buf)
                    buf = ""
                else:
                    buf += next_chr

            except:
                if DEBUG: print("Couldn't decode something with unicode in message: %s" %buf)
                #  print("got an exception")
            #print("we don't care")
                


############################################   SERVER PART    #################################################

# Now create server
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html', status="Welcome!")

# the msg HAS to be in this format for the server-side events to work
# the string must be "data: 'some string'\n\n" otherwise IT WON'T WORK...(I should probably use websockets at some point)
#
# here we're just reading the next availible string to send over from the thread-safe queue and yielding it to the streaming
# function hello, which flask handles for us
def getDataStream():
    while True:
        msg = serial_queue.get(True) # blocking if/when queue is empty
        yield "data: %s\n\n"%(msg)

# push the each message that is yielded 
@app.route('/stream')
def hello():
    if DEBUG: print("\n\nWE'RE SENDING STUFF YOUR WAY\n\n")
    return Response(stream_with_context(getDataStream()), mimetype="text/event-stream")


if __name__ == "__main__":
    teensy = get_teensy()
    names, num_report = get_vars(srcname)
    #run_serial(teensy, names, num_report)
    print(_thread.start_new_thread(run_serial, (teensy, names, num_report, ) ) )
    app.run(debug=True, threaded=True, use_reloader=False)

