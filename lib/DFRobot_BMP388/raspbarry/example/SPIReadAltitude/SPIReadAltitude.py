# Connect bmp388 and esp32 via SPI.
#
# Altitude is calculated based on temperature and sea level pressure
# The example can count an approximate altitude.
# Formula:
# P=P0*(1-H/44300)^5.256
#
# Warning:
#   This demo only supports python3.
#   Run this demo : python3 SPICountAltitude.py.
#
# connect:
#   raspberry       bmp388
#   CS  (15)        CSB
#   3.3v(17)        VCC
#   MOSI(19)        SDI
#   MISO(21)        SDO
#   SCLK(23)        SCK
#   GND (25)        GND

import bmp388
import time

# Define chip selection pins
cs = 22

# Create a bmp388 object to communicate with SPI.
bmp388 = bmp388.DFRobot_BMP388_SPI(cs)
time.sleep(0.5)

# You can use an accurate altitude to calibrate sea level air pressure. 
# And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
# In this case,525.0m is chendu accurate altitude.
seaLevel = bmp388.readSeaLevel(525.0);
print("seaLevel : %s Pa" %seaLevel)

# If there is no need to calibrate altitude, calibrated_altitude = False
calibrated_altitude = True

# Read pressure and count altitude.
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
