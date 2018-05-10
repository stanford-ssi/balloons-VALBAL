import sys
import numpy as np
import glob
import struct
import numpy
import serial
import csv
import pickle
import time 
head = "ALTITUDE_BAROMETER,ASCENT_RATE,LAS_STATE.v,LAS_STATE.effort,LAS_STATE.status,LAS_STATE.action,VALVE_QUEUE,BALLAST_QUEUE"
timestr = time.strftime("%Y%m%d-%H%M%S")
print(timestr)
f= open("test-outputs/SHTIL-" + timestr + ".csv","w+")
f.write(head+"\n")
f.close()

FSTART      = b'\xaa'

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

#print(serial_ports())
#exit()

if __name__ == '__main__':

	dat = np.load('ssi63bmp.npy')

	teensy = serial.Serial('/dev/ttyACM0')
	#wait for teensy configured for SITL testing
	while(1):
		read = teensy.read(1)

		if read == FSTART:
			print('>>> VALBAL found ready for SITL testing')
			teensy.write(FSTART)
			break
		else:
			try:
				print(read.decode("utf-8"),end='')
			except:
				pass
	while(1):
		read = teensy.read(1)
		if read == FSTART:
			print('>>> Data requested')
			request = teensy.read(4)
			time = struct.unpack('I',request)[0]/1000
			print('>>> Request Time:',time)
			idx = np.searchsorted(dat[:,0], time, side="left")
			print('>>> Returned Time:',dat[idx,0])
			data = np.flip(dat[idx,1:],axis=0)       #whoops had to flip it cause temp is first
			fetch = struct.pack('ffffffff',*data)
			teensy.write(fetch)	
			dat = dat[idx:,:]
			sta = teensy.read(9*4)
			status = struct.unpack('fffffffff',sta)
			print('>>> VB Status:', status)
			f= open("test-outputs/SHTIL-" + timestr + ".csv","a+")
			f.write(", ".join(str(i) for i in status) + "\n")
			f.close()

		else:
			try:
				pass
				print(read.decode("utf-8"),end='')
			except:
				pass