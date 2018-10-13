# Connect bmp388 and esp32 via I2C.
#
# Altitude is calculated based on temperature and sea level pressure
# The example can count an approximate altitude.
# Formula:
# P=P0*(1-H/44300)^5.256
#
# Warning:
#   This demo only supports python3.
#   Run this demo: python3 I2CCountAltitude.py.
#
# connect:
#   raspberry       bmp388
#   3.3v(1)         VCC
#   GND(6)          GND
#   SCL(5)          SCL
#   SDA(3)          SDA

import bmp388
import time

# Create a bmp388 object to communicate with I2C.
bmp388 = bmp388.DFRobot_BMP388_I2C()
time.sleep(0.5)

# You can use an accurate altitude to calibrate sea level air pressure. 
# And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
# In this case,525.0m is chendu accurate altitude.
seaLevel = bmp388.readSeaLevel(525.0);
print("seaLevel : %s Pa" %seaLevel)

# If there is no need to calibrate altitude, calibrated_altitude = False
calibrated_altitude = True

while 1:
  if(calibrated_altitude):
    # Read the calibrated altitude 
    altitude = bmp388.readCalibratedAltitude(seaLevel)
    print("calibrate Altitude : %s m" %altitude)
  else:
    # Read the altitude 
    altitude = bmp388.readAltitude();
    print("Altitude : %s m" %altitude)
  time.sleep(0.5)
