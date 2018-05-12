import numpy as np
import matplotlib.pyplot as plt

data = np.load('ssi63bmp.npy')
dats = data[:20*60*60*10,:]
np.save('ssi63bmp10hr',dats)
#plt.plot(dats[:,0],dats[:,1])
#plt.show()