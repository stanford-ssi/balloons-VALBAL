# Connect bmp388 and esp32 via IIC.
# Download bmp388.py and downloadAndRun this demo.

import bmp388
import time
from machine import Pin,I2C

# Create I2C object
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)

# Create a bmp388 object to communicate with I2C.
bmp388 = bmp388.DFRobot_BMP388_I2C(i2c)

# Read temperature and print it.
while 1:
  temp = bmp388.readTemperature()
  print("Temperature : %s C" %temp)
  time.sleep(0.5)
