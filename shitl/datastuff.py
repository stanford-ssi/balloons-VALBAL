import numpy as np
import matplotlib.pyplot as plt


df = pd.read_hdf('ssi63.h5')
[print(k) for k in df.keys()]
dfn = df[['raw_pressure_1','raw_pressure_2','raw_pressure_3','raw_pressure_4','raw_temp_1','raw_temp_2','raw_temp_3','raw_temp_4','lat_gps','long_gps','altitude_gps','heading_gps','speed_gps','num_sats_gps']]
dfn.reindex((dfn.index - df.index[0])/np.timedelta64(1, 's'))
data = np.hstack((((dfn.index - dfn.index[0])/np.timedelta64(1, 's')).values.reshape(-1,1),df.values))
np.save('ssi63shitl',data)
data[:20*60*60*10,:]
dats = np.save('ssi63shitl10hr',dats)