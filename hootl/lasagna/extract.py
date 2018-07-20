import struct
import pandas as pd
import numpy as np

L = 8742380
df = pd.read_hdf('ssi63.h5', start=0, stop=20*60*60*10)
print(len(df))

vrs = ['ALTITUDE_BAROMETER']
typ = ['f' for _ in range(len(vrs))]


#vrs.extend(['SPAG_EFFORT', 'SPAG_VENT_TIME_INTERVAL', 'SPAG_BALLAST_TIME_INTERVAL', 'SPAG_VALVE_INTERVAL_COUNTER', 'SPAG_BALLAST_INTERVAL_COUNTER', 'ACTION_SPAG', 'SPAG_VENT_TIME_TOTAL', 'SPAG_BALLAST_TIME_TOTAL'])
vrs = [vr.lower() for vr in vrs]
#typ.extend(['f','f','f','I','I','i','I', 'I'])

assert len(vrs) == len(typ)
m = {'f': 'float', 'i': 'int', 'I': 'unsigned int'}

open("header.h","w").write("typedef struct __attribute__ ((packed)) {\n"+'\n'.join(["  "+m[t] +" "+v.upper()+";" for t, v in zip(typ, vrs)])+"\n} miniframe;\n")

S = '<' + ''.join(typ)

f = open("data.bin","wb")

for i in range(len(df)):
    if i % 10000 == 0: print(float(i)/len(df)*100)
    row = df.iloc[i]
    vals = [row[vr] for vr in vrs]
    #with pd.option_context('display.max_rows', None, 'display.max_columns', 3):
    #    print row
    #print df['spag_effort'][i]
    """if vals[-3] < 0:
        print df['spag_vent_time_total'].values.dtype, np.min(df['spag_vent_time_total']), "was the min!"
        print "wot", i, vals"""
    d =  struct.pack(S, *vals)
    #print len(d)
    f.write(d)
    if i % 10000 == 0: f.flush()

f.close()
