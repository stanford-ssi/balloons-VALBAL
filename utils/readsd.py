import os.path
import sys
import struct
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import re
from pprint import pprint as pp

d = os.path.dirname(os.path.abspath(__file__))
#with open(os.path.abspath(os.path.join(d, "../src/Data.h"))) as ff:
with open(os.path.abspath(os.path.join(d, "frame2.h"))) as ff:
    t = ff.read()
    stuffs = re.findall('^  (.*?)\s+(.*?)\s+=',t, re.MULTILINE)

#f = open(sys.argv[1])
f = open('/dev/mmcblk0','rb')
#f = open('/home/joan/image.sdcard','rb')

N = 512

data = []

#sz = os.path.getsize(sys.argv[1])

#while True:
#for _ in range(87744):
#    f.read(N)

a = 45070
b = 143100
d = b-a
aa = {"uint32_t": "I", "float": "f", "uint16_t": "H", "bool": "?", "uint8_t": "B", "int32_t":"I"}
sz = {"uint32_t": 4, "float": 4, "uint16_t": 2, "bool": 1, "uint8_t": 1, "int32_t": 4}
mp = {"uint32_t": 'uint32', "float": 'float32', "uint16_t": 'uint16', "bool": 'bool_', "uint8_t": 'uint8',"int32_t":"int32"}
mp2 = {"uint32_t": '<u4', "float": '<f4', "uint16_t": '<u2', "bool": 'u1', "uint8_t": 'u1'}
typestr=[(x[1].lower(),mp[x[0]]) for i, x in enumerate(stuffs)]
A = np.zeros((d,), dtype=typestr)
stuffs = [(k.lower(), v.lower()) for (k, v) in stuffs]

alts = []
lops = []
for _ in range(45070):
    f.read(1024)
for _ in range(d):
    #print(_)
    frame = f.read(1024)
    #print(frame)
    if not frame: break

    df = {}
    i = 0
    for (t, n) in stuffs:
        df[n] = struct.unpack(aa[t], frame[i:i+sz[t]])[0]
        A[_][n] = df[n]
        i += sz[t]
    #if len(data) > 0 and abs(df['TIME']-data[-1]['TIME']) > 1000*10: break
    pp((df['loop_number'], _))
    continue
    #pp(df['LOG_TIME'])
    #if len(data) > 0 and df['LOOP_NUMBER'] - data[-1]['LOOP_NUMBER'] != 1:
    #    print(df['LOOP_NUMBER']-data[-1]['LOOP_NUMBER'])
    #print(_, df['LOOP_NUMBER'])
    if len(lops)>= 10 and (df['LOOP_NUMBER'] == 0) and lops[-1] == 0 and lops[-9] == 0: break
    #if len(data)>= 10 and (df['LOOP_NUMBER'] == 0) and data[-1] == 0 and data[-9] == 0: break
    #if len(data)>= 2 and (df['LOOP_NUMBER'] > data[-1]['LOOP_NUMBER'] + 20): break
    if df['LOOP_NUMBER'] >= 20*86400*10:
        continue

    #data.append(df)
    lops.append(df['LOOP_NUMBER'])
    alts.append(df['ALTITUDE_BAROMETER'])
    continue
    data.append(df['LOOP_NUMBER'])
    alts.append(df['ALTITUDE_BAROMETER'])

df = pd.DataFrame.from_records(A, index='time')#, columns=[x[0].lower() for x in stuffs])
launch_unix = 1524345300 * 1000
df.index = pd.to_datetime(df.index + launch_unix, unit='ms')
df.to_hdf('ssi66.h5', 'df', complib='zlib', mode='w', complevel=5)
print("k done")
exit()

#import pickle
#pickle.dump(data, open('pickled3','wb'))
#pp(data[-1])
#log = np.array([x['LOOP_NUMBER'] for x in data])
#log = np.array([x['CPU_SPEED'] for x in data])
import matplotlib.pyplot as plt
#plt.plot(log)
plt.plot(alts)
plt.show()
plt.plot(lops)
plt.show()
print(log.mean(),log.max(), log.min())
exit()
"""
lop = np.array([x['LOOP_TIME'] for x in data])
print 1000./np.mean(lop)
ln = [x['LOOP_NUMBER2'] for x in data]
#print ln
d = np.diff(ln)
print (np.where(d != 1))#, np.array(ln)[np.where(d!=1)]
print d.mean(), d.max(), d.min()
exit()
pp("-------")
pp(data[6127])
pp("-------")
pp(data[6128])
pp("-------")
pp(data[6129])
exit()
loops = loops[:-1]
numbers = numbers[:-1]
stuffs = stuffs[:-1]
time =  stuffs[-1]/1000./60./60.
print "Test time", stuffs[-1]/1000./60., "minutes"
print "Running in non-safe mode"
stuffs = np.diff(np.array(stuffs))
loops = np.array(loops[:-1])
numbers = np.diff(numbers)
print "Loop times", "mean", stuffs.mean(), "max", stuffs.max(), "min", stuffs.min()
print "SD time","mean",loops.mean(),"max",loops.max(), "min",loops.min()
print "Number of times loop time > 6ms:", len(filter(lambda x: x>6, loops))
print "Frequency of loop time fuckups", len(filter(lambda x: x>6, loops))/float(time), "per hour"
#plt.hist(filter(lambda x: x>6, loops),bins=100)
#plt.show()
print "Data integrity", "yep" if np.all(numbers==1) else "nope" #numbers.min(), numbers.max(), numbers.mean()
"""
