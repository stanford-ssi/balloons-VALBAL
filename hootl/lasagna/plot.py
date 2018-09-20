import numpy as np 
import struct
import matplotlib.pyplot as plt
import matplotlib
import time
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
t = np.arange(0,data.shape[0])

v = ['alt', 'cmd', 'ef', 'ef_s','fuse','v','act']
D = lambda x : data[:,v.index(x)]
fig, ax1 = plt.subplots()
ax1.plot(t,D('alt'),label='alt')
ax1.set_ylabel('altitude (m)')

if 0:
	for i in np.nonzero(np.diff(D('act')) > 0)[0]:
		ax1.axvline(t[i], c='b',alpha=0.01)
	for i in np.nonzero(np.diff(D('act')) < 0)[0]:
		ax1.axvline(t[i], c='g',alpha=0.2)

ax1.axhline(14250,c='gray',alpha=0.4)
ax1.axhline(12750,c='gray',alpha=0.4)

print(np.sum(np.abs(np.diff(D('act')))))

ax2 = ax1.twinx()
ax2.plot(t,D('fuse'),'red',label='velocity')
ax2.plot(t,D('v'),'orange',label='fused velocity')
#ax2.set_ylabel('velocity')
#ax1.set_title(np.sum(np.abs(np.diff(D('act')))))
#ax3 = ax1.twinx()
#ax3.plot(t,D('ef'),'pink',label='effort')
formatter = matplotlib.ticker.FuncFormatter(lambda s, x: time.strftime('%d:%H:%M:%S', time.gmtime(s // 1)))
ax1.xaxis.set_major_formatter(formatter)
lines, labels = ax1.get_legend_handles_labels()
#lines2, labels2 = ax2.get_legend_handles_labels()
#ax1.legend(lines + lines2, labels + labels2, loc=5)
plt.show()
