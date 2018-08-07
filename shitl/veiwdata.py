import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import pysolar as ps

df = pd.read_csv("test-outputs/SHITL56.csv")
print(df.keys()[-8])

for time in df.index:
	ps.get_altitude_fast(time)
plt.subplot(311)
plt.plot(df.index/60/60,df[' data.SOLAR_ELEVATION'].values)
plt.ylabel("solar elevation")
plt.subplot(312)
plt.plot(df.index/60/60,df[' data.DSEDT'].values)
plt.ylabel("dsedt")
plt.subplot(313)
plt.plot(df.index/60/60,df[' data.ESTIMATED_DLDT'].values)
plt.ylabel("estimated dldt")
plt.show()