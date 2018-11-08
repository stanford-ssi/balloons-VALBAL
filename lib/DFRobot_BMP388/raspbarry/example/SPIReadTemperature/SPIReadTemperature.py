# Connect bmp388 and esp32 via SPI.
#
# Warning:
#   This demo only supports python3.
#   Run this demo : python3 SPIReadTemperature.py
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

# Read temperature and print it.
while 1:
  temp = bmp388.readTemperature()
  print("Temperature : %s C" %temp)  
  time.sleep(0.5)