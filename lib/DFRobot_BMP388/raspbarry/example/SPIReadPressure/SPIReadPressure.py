# Connect bmp388 and esp32 via SPI.
#
# Warning:
#   This demo only supports python3.
#   Run this demo : python3 SPIReadPressure.py.
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

# Read pressure and print it.
while 1:
  pres = bmp388.readPressure()
  print("Temperature : %s pa" %pres)  
  time.sleep(0.5)