import numpy as np
import matplotlib.pyplot as plt
import sys

dat = np.fromfile(sys.argv[1], dtype=np.float32)
print(dat, dat.shape)
plt.plot(dat)
plt.show()
