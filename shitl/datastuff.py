import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

np.set_printoptions(edgeitems=1000)
if 0:
	dat = (np.load('ssi71shitl10hr.npy'))
	print(int(dat[0,-1]))
	exit()

df = pd.read_hdf('../../flightdata/fulldf/ssi71.h5')
[print(k) for k in df.keys()]
dfn = df[['raw_pressure_1','raw_pressure_2','raw_pressure_3','raw_pressure_4','raw_temp_1','raw_temp_2','raw_temp_3','raw_temp_4','lat_gps','long_gps','altitude_gps','heading_gps','speed_gps','num_sats_gps']]
del df
dfn = dfn.iloc[:20*60*60*10]
dfn.reindex((dfn.index - dfn.index[0])/np.timedelta64(1, 's'))
data = np.hstack((((dfn.index - dfn.index[0])/np.timedelta64(1, 's')).values.reshape(-1,1),dfn.values))
np.save('ssi71shitl10hr',data)
