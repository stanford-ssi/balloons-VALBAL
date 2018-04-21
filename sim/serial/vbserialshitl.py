import serial
import sys
import struct
import pandas as pd

BAUD = 9600

assert len(sys.argv) == 3
PORT = sys.argv[1]
DATA_PATH = sys.argv[2]
START_SIGNAL = b'SERIAL_SHITL_BEGIN'
REQUEST_SIGNAL = b'SERIAL_SHITL_REQUEST'

print("\n>>> Starting data import")
data = pd.read_csv(DATA_PATH)
print("\n>>> Data imported succesfully")
print("\n>>>Opening serial port")
vb = serial.Serial(PORT, BAUD, timeout=1)
print("\n>>> Port opened succcesfully")

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
		data_row = data.ix[(data['Time'] - requested_time).abs().argsort()[:2]]
	
		vb.write(struct.pack('f', float(list(data_row['raw_temp_1'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_temp_2'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_temp_3'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_temp_4'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_pressure_1'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_pressure_2'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_pressure_3'])[0])))
		vb.write(struct.pack('f', float(list(data_row['raw_pressure_4'])[0])))		
	else: 
		print(incoming)

