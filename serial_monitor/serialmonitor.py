import serial
import struct
import time as py_time
import sys
import numpy as np
from sys import argv, exit 
import re
from websocket import create_connection


from flask import Flask, render_template, Response, stream_with_context
from flask_socketio import SocketIO
import queue
import _thread

# constants
TEENSY_ADR = '/dev/tty.usbmodem144121'#'/dev/ttyACM0'
srcname = "../src/Shitl.cpp"
log_file_path = "sample_shitl.txt"
FSTART = b'\xaa'
EXTRA_START = ';'

# flags
WILL_LOG = False#True
READ_LOG = True

# global var... don't try this at home
serial_queue = queue.Queue()

# setup log file stuff
if WILL_LOG and READ_LOG:
    print("CANNOT READ AND WRITE TO LOG FILE AT THE SAME TIME")
    exit(1)
if WILL_LOG: log_file = open(log_file_path, 'ab')
if READ_LOG: log_file = open(log_file_path, 'rb')

# more organized code
def get_teensy():
    if READ_LOG:
        return log_file
    else:
        return serial.Serial(TEENSY_ADR)


def get_vars(srcname):
    names = []
    num_report = 0
    f = open(srcname, 'r')
    lines = f.readlines()
    for i,line in enumerate(lines):
        if re.search('diddlybop', line):
            num_report = int(re.findall(r"\[(.*?)\]",lines[i+1])[0])
            names = [re.findall(r"\= (.*?)\;",k)[0] for k in lines[i+2:i+2+num_report]]
            print(names)
            break
    f.close()
    names.insert(0,'time')
    return (names, num_report)

def send_message(message):
    #global serial_queue
    serial_queue.put(message)

def run_serial(teensy, names, num_report):
    # Await teensy response to make sure we can talk
    # But only do this if it's the first time reading in serial
    buf = ""
    using_extra = False
    while (1):
        #print('>>> about to read start byte')
        read = teensy.read(1)

        if WILL_LOG: log_file.write(read)
        #print('>>> read start byte')
        if read == FSTART:
            print('>>> VALBAL found ready to monitor serial')

            if not READ_LOG: teensy.write(FSTART) # if the serial monitor is re-run, it will send an FSTART regardless of if the Teensy is expecting it, 
                                 # so make sure FSTARTs are dealt with on the teensy side in an appropriate manner
            break
        else:
            #break # just for debugging
            try:
                pass
                #print(read.decode("utf-8"),end='')
            except:
                pass

    # Read teensy data
    print('reading data')
    while(1):
        if READ_LOG: py_time.sleep(0.001)
        #print('>>> Reading byte:')
        #if (len(argv) > 2):
        #    read = teensy.read(4)
        read = teensy.read(1)

        if WILL_LOG: log_file.write(read)

        if read == FSTART:
            print('>>> Ready to read VALBAL data')
            request = teensy.read(4) #takes number of bytes read
            time = struct.unpack('I', request)[0]/1000

            print('>>> Request Time:',time)
            if WILL_LOG: log_file.write(request)
            
            # read the rest of the data:
            data = teensy.read(4*num_report)

            status = list(struct.unpack('f'*num_report, data))
            status.insert(0, time)

            # if the buffer isn't empty, just flush it
            if (len(buf) > 0):
                #print(buf)
                send_message(buf)
                buf = ""

            
            #print(">>> VB Status: ", status)
            print(">>> VB Status:")
            for i in range(num_report):
                buf += "\t%s:\t%f;"%(names[i], status[i])
                print("\t%s:\t%f"%(names[i], status[i]))
            if WILL_LOG: log_file.write(data)
            buf = buf[:-1] # shave off the last semicolon
            send_message(buf)
            buf = ""

        else:
            try:
                if (len(buf) == 0):
                    buf = EXTRA_START

                next_chr = read.decode("utf-8")
                if (next_chr == "\n"):
                    print(buf)
                    send_message(buf)
                    buf = ""
                else:
                    buf += next_chr
               # print(next_chr, end="")

                #pass
                #if len(buffer) > 100:
                #    print("going to send a message")
                #print(len(buffer))
                #if len(buffer) > 1000:
                #    send_message(buffer)#"Hello there!")#read.decode("utf-8"))
                #    buffer = ""
                #else:
                #    buffer += read.decode("utf-8")
                #print("other thing: ")
                #print(read.decode("utf-8"), end='')
            except:
                pass

#if __name__ == "__main__":
    #teensy = get_teensy()
    #names, num_report = get_vars(srcname)
    #run_serial(teensy, names, num_report)
#    send_message("Hello there")

#run_serial()
# Now actually run the stuff
#try:
#    _thread.start_new_thread(run_serial)
#    _thread.start_new_thread(gui.run_app)
#except:
#    print("Error: could not start threads")
#
#while True:
#    pass
#gui.run_app()
#




# Now create server
app = Flask(__name__)
#socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template('index.html', status="Welcome!")

def getDataStream():
    #c = 0
    while True:
        msg = serial_queue.get(True)
        #msg = c + 1
        #c += 1
        #py_time.sleep(1)
        yield "data: %s\n\n"%(msg)

@app.route('/stream')
def hello():
    print("\n\nHELLO THERE MY PRETTY\n\n")
    return Response(stream_with_context(getDataStream()), mimetype="text/event-stream")

#@socketio.on('message', namespace = "/echo")
#def echo_socket():
#    while not ws.closed:
#        message = ws.receive()
#        ws.send(message)

if __name__ == "__main__":
    teensy = get_teensy()
    names, num_report = get_vars(srcname)
    _thread.start_new_thread(run_serial, (teensy, names, num_report, ) )
    #socketio.run(app)
    app.run(debug=True, threaded=True)
