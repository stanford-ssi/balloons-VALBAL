import re
import struct
import os

with open("commands.txt","r") as f:

	lines = f.readlines()[0]
	cmd = re.findall(r"\((.*?)\)",lines)[0].split(',')
	index = int(cmd[0])
	value = float(cmd[1])
	print(chr(index))
	cmd_msg = struct.pack('if',*[index,value])
	print(cmd_msg)
