import os.path
import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
import re
from pprint import pprint as pp

d = os.path.dirname(os.path.abspath(__file__))
with open(os.path.abspath(os.path.join(d, "../src/Data.h"))) as ff:
    t = ff.read()
    stuffs = re.findall('^  (.*?)\s+(.*?)\s+=',t, re.MULTILINE)

f = open(sys.argv[1])

N = 512

data = []

sz = os.path.getsize(sys.argv[1])

while True:
    frame = f.read(N)
    if not frame: break

    df = {}
    i = 0
    aa = {"uint32_t": "I", "float": "f", "uint16_t": "H", "bool": "?", "uint8_t": "B"}
    sz = {"uint32_t": 4, "float": 4, "uint16_t": 2, "bool": 1, "uint8_t": 1}
    for (t, n) in stuffs:
        df[n] = struct.unpack(aa[t], frame[i:i+sz[t]])[0]
        i += sz[t]
    if len(data) > 0 and abs(df['TIME']-data[-1]['TIME']) > 1000*10: break
    if len(data) > 0 and df['LOOP_NUMBER'] - data[-1]['LOOP_NUMBER'] != 1:
        print df['LOOP_NUMBER']-data[-1]['LOOP_NUMBER']
    data.append(df)
#pp(data[-1])
log = np.array([x['LOG_TIME'] for x in data])
print log.mean(),log.max(), log.min()
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
