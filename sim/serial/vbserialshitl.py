import serial
import sys
import struct
import pandas as pd

BAUD = 9600

assert len(sys.argv) >= 3
output = False
if len(sys.argv) == 4:
	output = True
PORT = sys.argv[1]
DATA_PATH = sys.argv[2]
if output:
	OUTPUT_PATH = sys.argv[3]
	search_list = [b"numExecNow:", b"Altitude:", b"RAW_PRESSURE_1:", b"RAW_PRESSURE_2:", b"RAW_PRESSURE_3:", b"RAW_PRESSURE_4:"]
	output_index = 0

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
	elif output and len(incoming.split()) == 2 and incoming.split()[0] in search_list:
		if incoming.split()[0] == search_list[output_index]:
			with open(OUTPUT_PATH, "a") as d_out:
				d_out.write(str(float(incoming.split()[1])) + ",")
			output_index += 1
			if output_index == len(search_list):
				with open(OUTPUT_PATH, "a") as d_out:
					d_out.write("\n")
				output_index = 0
		print(incoming)
	else: 
		print(incoming)