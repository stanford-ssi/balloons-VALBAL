import numpy as np
import matplotlib.pyplot as plt
import sys
import datetime as dt
import pytz
import pysolar as ps


d = dt.datetime(2018, 1, 1, 21, 0, 0, tzinfo=pytz.utc)
d = dt.datetime(1, 1, 1, 0, 0, 0, tzinfo=pytz.utc)

se = []
se2 = []
for s in range(60*60*10):
#	se.append(ps.solar.get_altitude(37.4275,-122.1697,d+dt.timedelta(seconds=s)))
	se2.append(ps.solar.get_altitude_fast(37.4275,122.1697,d+dt.timedelta(seconds=s)))

dat = np.fromfile("output.bin", dtype=np.float64)
dat = dat.reshape(-1,3)

plt.subplot(3,1,1)
plt.plot(dat[:,0])
plt.plot(se2)
plt.grid()
plt.subplot(3,1,2)
plt.plot(dat[:,1])
plt.grid()
plt.subplot(3,1,3)
plt.plot(dat[:,2])
plt.grid()
plt.show()