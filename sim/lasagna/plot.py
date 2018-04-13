import numpy as np 
import struct
import matplotlib.pyplot as plt

data = []
with open("output.bin", "rb") as f:
	while True:
		dat = f.read(4*7)
		if dat:
			b = struct.unpack('ffffffi',dat)
			data.append(b)
		else:
			break

data = np.array(data)
t = np.arange(0,data.shape[0])/20/60/60

v = ['alt', 'cmd', 'ef', 'ef_s','fuse','v','act']
D = lambda x : data[:,v.index(x)]
print(D('fuse'))
plt.plot(D('ef_s'))
plt.show()
exit()
fig, ax1 = plt.subplots()
ax1.plot(t,D('alt'))

for i in np.nonzero(np.diff(D('act')) > 0)[0]:
	ax1.axvline(t[i], c='b',alpha=0.2)
for i in np.nonzero(np.diff(D('act')) < 0)[0]:
	ax1.axvline(t[i], c='g',alpha=0.2)


ax1.axhline(14750,c='gray',alpha=0.4)
ax1.axhline(13250,c='gray',alpha=0.4)

print(np.sum(np.abs(np.diff(D('act')))))
'''	
ax2 = ax1.twinx()
ax2.plot(t,D('ef'),'black')
,alpha=0.4
ax3 = ax1.twinx()
ax3.plot(t,D('fuse'),'red')
ax3.plot(t,D('v'),'orange')
'''
ax1.set_title(np.sum(np.abs(np.diff(D('act')))))
plt.show()
