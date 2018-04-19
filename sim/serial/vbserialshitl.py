import serial
import sys
import struct

BAUD = 9600

assert len(sys.argv) == 3
PORT = sys.argv[1]
DATA_PATH = sys.argv[2]
START_SIGNAL = b'SERIAL_SHITL_BEGIN'
REQUEST_SIGNAL = b'SERIAL_SHITL_REQUEST'

with open(DATA_PATH, 'rb') as f:
	data = None
	# TODO: read data
	
vb = serial.Serial(PORT, BAUD, timeout=1)

while(False):
	incoming = vb.readline().strip()
	print(incoming)
	if incoming == START_SIGNAL:
		print('\n>>> ValBal ready for SHITL testing')
		# TODO: Have AV wait for command singal form simulation
		break

while(True):
	incoming = vb.readline().strip()
	if len(incoming.split()) == 2 and incoming.split()[0] == REQUEST_SIGNAL:
		requested_time = float(incoming.split()[1])
		print("\n>>> 	ValBal Requested Data at Time: " + str(requested_time))
		first = struct.pack('f', 1.1)
		second = struct.pack('f', 2.2)
		third = struct.pack('f', -3.3)
		fourth = struct.pack('f', requested_time)
		fifth = struct.pack('f', 5.5)
		sixth = struct.pack('f', 6666.666)
		seventh = struct.pack('f', -7777777.77)
		eigth = struct.pack('f', 8)
		vb.write(first)
		vb.write(second)
		vb.write(third)
		vb.write(fourth)	
		vb.write(fifth)
		vb.write(sixth)	
		vb.write(seventh)
		vb.write(eigth)			
	else: 
		# print(incoming.decode("utf-8"),end='')
		print(incoming)

