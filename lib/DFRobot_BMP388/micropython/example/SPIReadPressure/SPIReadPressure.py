# Connect bmp388 and esp32 via SPI.
# Connect CSB pin to IO26/D3 of esp32
# Download bmp388.py and downloadAndRun this demo.

import bmp388
from machine import Pin,SPI
import time

# Chip selection pin D3/pin26
D3 = 26

# Create SPI object
spi = SPI(baudrate=100000, polarity=1, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))

# Set the chip selection pin to output.
cs = Pin(D3,Pin.OUT)

# Create a bmp388 object to communicate with SPI.
bmp388 = bmp388.DFRobot_BMP388_SPI(spi,cs)

# Read pressure and print it.
while 1:
  pressure = bmp388.readPressure();
  print("Pressure : %s Pa" %pressure)
  time.sleep(0.5)
