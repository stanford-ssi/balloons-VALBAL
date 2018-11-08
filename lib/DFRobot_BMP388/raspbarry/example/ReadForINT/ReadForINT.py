# Connect bmp388 and esp32 via I2C/SPI.
#
# Warning:
#   This demo only supports python3.
#   Run this demo: python3 ReadForINT.py.
#
# connect: 
#   raspberry       bmp388
#   13              INT
#(I2C)
#   3.3v(1)         VCC
#   GND(6)          GND
#   SCL(5)          SCL
#   SDA(3)          SDA
#
#(SPI)
#   raspberry       bmp388
#   CS  (15)        CSB
#   3.3v(17)        VCC
#   MOSI(19)        SDI
#   MISO(21)        SDO
#   SCLK(23)        SCK
#   GND (25)        GND
# 
import bmp388
import time
import RPi.GPIO as GPIO
# If 0, connect BMP388 to SPI interface of raspberry, else connect I2C interface
if 0:
  # Create a bmp388 object to communicate with I2C.
  bmp388 = bmp388.DFRobot_BMP388_I2C()
  
else:
  # Define chip selection pins
  cs = 22

  # Create a bmp388 object to communicate with SPI.
  bmp388 = bmp388.DFRobot_BMP388_SPI(cs)

INT = 27 
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT, GPIO.IN)

flag = 0
def func(v):
  global flag
  flag = 1
  
bmp388.INTEnable()

GPIO.add_event_detect(INT,GPIO.RISING,callback=func,bouncetime=200)
t = time.time()
# Read pressure and print it.
while 1:
  if(flag == 1):
    temp = bmp388.readTemperature()
    print("Temperature : %s C" %temp)
    pres = bmp388.readPressure()
    print("Pressure : %s Pa" %pres)
    flag = 0
    time.sleep(0.5)
  if(time.time()-t>10):
    bmp388.INTDisable()