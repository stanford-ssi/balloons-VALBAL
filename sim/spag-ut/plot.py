import numpy as np 
import struct
import matplotlib.pyplot as plt

data = []
with open("output.bin", "rb") as f:
	while True:
		dat = f.read(4*6)
		if dat:
			b = struct.unpack('ffffff',dat)
			data.append(b)
		else:
			break
data = np.array(data)
t = np.arange(0,data.shape[0])/20/60/60
print(data)
v = ['alt', 'cmd', 'ef','act', 'fuse','v']
D = lambda x : data[:,v.index(x)]
print(D('act'))

fig, ax1 = plt.subplots()
ax1.plot(t,D('alt'))
ax1.plot(t,D('cmd'))
for i in np.nonzero(D('act') > 0)[0]:
	ax1.axvline(t[i], c='b',alpha=0.2)
for i in np.nonzero(D('act') < 0)[0]:
	ax1.axvline(t[i], c='g',alpha=0.2)


ax2 = ax1.twinx()
ax2.plot(t[50000:],D('ef')[50000:])

ax3 = ax1.twinx()
ax3.plot(t[100000:],D('fuse')[100000:],'red')
ax3.plot(t[100000:],D('v')[100000:],'orange')

plt.show()
