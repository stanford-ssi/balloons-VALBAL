import serial
import numpy as np
import sys
import struct
import re
from pprint import pprint as pp
import time

if len(sys.argv) != 2:
    print('pls block number')
    exit(1)

with open("Data.h") as ff:
    t = ff.read()
    stuffs = re.findall('^\s*(.*?)\s+(.*?)\s*[=;]',t, re.MULTILINE)

data = []

aa = {"uint32_t": "I", "float": "f", "uint16_t": "H", "bool": "?", "uint8_t": "B", "int32_t":"I"}
sz = {"uint32_t": 4, "float": 4, "uint16_t": 2, "bool": 1, "uint8_t": 1, "int32_t": 4}
mp = {"uint32_t": 'uint32', "float": 'float32', "uint16_t": 'uint16', "bool": 'bool_', "uint8_t": 'uint8',"int32_t":"int32"}
mp2 = {"uint32_t": '<u4', "float": '<f4', "uint16_t": '<u2', "bool": 'u1', "uint8_t": 'u1'}
typestr=[(x[1].lower(),mp[x[0]]) for i, x in enumerate(stuffs)]
stuffs = [(k.lower(), v.lower()) for (k, v) in stuffs]


s = serial.Serial('/dev/ttyACM0')

s.write('\xaa')

sys.argv[1] = int(sys.argv[1])

b0 = sys.argv[1] & 0x000000ff
b1 = (sys.argv[1] & 0x0000ff00) >> 8
b2 = (sys.argv[1] & 0x00ff0000) >> 16
b3 = (sys.argv[1] & 0xff000000) >> 24

s.write(chr(b0))
s.write(chr(b1))
s.write(chr(b2))
s.write(chr(b3))

t0 = time.time()
byt = 0
bl = ""
while True:
    if len(bl) == 1024:
        byt += 1024 + 8
        print(byt/(time.time()-t0)/1024., "KBps")
        df = {}
        i = 0
        for (t, n) in stuffs[:2]:
            df[n] = struct.unpack(aa[t], bl[i:i+sz[t]])[0]
            i += sz[t]    
        #print(df['loop_number'], df['time'], df['altitude_barometer'], df['action_time_totals6'], df['action_time_totals7'])
        bl = ""
    block = s.read(512)
    bl += block
    chk = s.read(4)
    c = np.uint32(0)
    for i in range(128):
        c += np.int32(i) * np.frombuffer(block[4*i:4*i+4], dtype=np.uint32)
    if c != np.frombuffer(chk, dtype=np.uint32):
        sys.stdout.write('X')
    else:
        sys.stdout.write('.')
