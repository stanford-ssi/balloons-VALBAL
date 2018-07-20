import numpy as np 
import struct
import matplotlib.pyplot as plt

data = []
with open("output.bin", "rb") as f:
	while True:
		dat = f.read(4*3)
		if dat:
			b = struct.unpack('ffi',dat)
			data.append(b)
		else:
			break

data = np.array(data)
t = np.arange(0,data.shape[0])/60/60

v = ['alt', 'ef', 'act']
D = lambda x : data[:,v.index(x)]
fig, ax1 = plt.subplots()
ax1.plot(t,D('alt'))

for i in np.nonzero(np.diff(D('act')) > 0)[0]:
	ax1.axvline(t[i], c='b',alpha=0.2)
for i in np.nonzero(np.diff(D('act')) < 0)[0]:
	ax1.axvline(t[i], c='g',alpha=0.2)


ax1.axhline(14750,c='gray',alpha=0.4)
ax1.axhline(13250,c='gray',alpha=0.4)

print(np.sum(np.abs(np.diff(D('act')))))

#ax3 = ax1.twinx()
#ax3.plot(t,D('fuse'),'red')
#ax3.plot(t,D('v'),'orange')

ax1.set_title(np.sum(np.abs(np.diff(D('act')))))
plt.show()
