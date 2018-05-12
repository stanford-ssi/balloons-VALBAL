import sys
import numpy as np
import glob
import struct
import numpy
import serial
import csv
import pickle
import time 
import re
import os
FSTART      = b'\xaa'
cmdtime = os.path.getmtime('commands.txt')
cmd_ready=False
srcname = "../src/Shitl.cpp"
file = open(srcname, 'r')
lines = file.readlines()
for i,line in enumerate(lines):
    if re.search('diddlybop', line):
    	num_report = int(re.findall(r"\[(.*?)\]",lines[i+1])[0])
    	names = [re.findall(r"\= (.*?)\;",k)[0] for k in lines[i+2:i+2+num_report]]
    	print(names)
    	break
names.insert(0,'time')
i = 0
while os.path.exists("test-outputs/SHITL%s.csv"%i):
    i += 1
logfile=  "test-outputs/SHITL%s.csv"%i
binfile= "test-outputs/SHITL%s.bin"%i
f= open(logfile,"w+")
f.write(", ".join(names)+"\n")
f.close()
fbin = open(binfile,"wb")
dat = np.load('ssi63bmp10hr.npy')
teensy = serial.Serial('/dev/ttyACM0')

#WAIT FOR TEENSY TO BEGIN TESTING
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

n = 0
while(1):
	read = teensy.read(1)
	if read == FSTART:
		
		#READ REQUEST
		print('>>> Data requested')
		request = teensy.read(4)

		## SEND FLAGS
		if cmd_ready:
			teensy.write(b'\x01')
		else:
			teensy.write(b'\x00')


		## SENDS DATA
		time = struct.unpack('I',request)[0]/1000
		print('>>> Request Time:',time)
		idx = np.searchsorted(dat[:,0], time, side="left")
		print('>>> Returned Time:',dat[idx,0])
		data = np.flip(dat[idx,1:],axis=0)       #whoops had to flip it cause temp is first
		fetch = struct.pack('ffffffff',*data)
		teensy.write(fetch)	
		
		## SEND ADDITIONAL DATA
		if cmd_ready:
			teensy.write(cmd_msg)
			cmd_ready=False;
			print('>>> Sent Command',index,value)

		## DO NON-TIME-CRITICAL THINGS
		dat = dat[idx:,:]
		sta = teensy.read(num_report*4)
		status = list(struct.unpack("f"*num_report,sta))
		status.insert(0,float(time))
		print('>>> VB Status:', status)
		if os.path.getmtime('commands.txt') != cmdtime:
			print(">>> Command file changed")
			cmdtime = os.path.getmtime('commands.txt')
			with open("commands.txt","r") as f:
				try:
					lines = f.readlines()[0]
					cmd = re.findall(r"\((.*?)\)",lines)[0].split(',')
					index = int(cmd[0])
					value = float(cmd[1])
					print(">>> ",index,value)
					cmd_msg = struct.pack('if',*[index,value])
					cmd_ready = True
				except:
					pass
		fbin.write(sta)
		if(n % (20*1) == 0):
			f= open(logfile,"a+")
			f.write(", ".join(str(i) for i in status) + "\n")
			f.close()
		n+=1

	else:
		try:
			pass
			print(read.decode("utf-8"),end='')
		except:
			pass