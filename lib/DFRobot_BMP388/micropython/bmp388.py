# From machine import Pin,I2C,SPI
import time
from math import pow
import sys
class DFRobot_BMP388:
  def __init__(self):
    self.op_mode = 0
    self.par_t1 = 0
    self.par_t2 = 0
    self.par_t3 = 0
    self.par_p1 = 0
    self.par_p2 = 0
    self.par_p3 = 0
    self.par_p4 = 0
    self.par_p5 = 0
    self.par_p6 = 0
    self.par_p7 = 0
    self.par_p8 = 0
    self.par_p9 = 0
    self.par_p10 = 0
    self.par_p11 = 0
    self.addr = 119
    chip_id = self.bmp3_get_regs(0x00, 1)[0]
    # Print(hex(chip_id))
    if (chip_id != 0x50):
      print("chip id error!")
      sys.exit()
    self.get_calib_data()
    self.set_config()
  
  def get_calib_data(self):
    calib = self.bmp3_get_regs(0x31,21) # Read calibration data.
    self.parse_calib_data(calib)
  def uint8_int(self,num):
    if(num>127):
      num = num - 256
    return num
  def parse_calib_data(self,calib): # Parse
    temp_var = 0.00390625
    self.par_t1 = ((calib[1]<<8)|calib[0])/temp_var
    
    temp_var = 1073741824
    self.par_t2 = ((calib[3]<<8)|calib[2])/temp_var
    
    temp_var = 281474976710656
    calibTemp = self.uint8_int(calib[4])
    self.par_t3 = (calibTemp)/temp_var
    
    temp_var = 1048576
    calibTempA = self.uint8_int(calib[6])
    calibTempB = self.uint8_int(calib[5])
    self.par_p1 = ((calibTempA|calibTempB)-16384)/temp_var
    # Print((calibTempA<<8)|calibTempB)
    
    temp_var = 536870912
    calibTempA = self.uint8_int(calib[8])
    calibTempB = self.uint8_int(calib[7])
    self.par_p2 = (((calibTempA<<8)|calibTempB)-16384)/temp_var
    # Print((calibTempA<<8)|calibTempB)
    
    temp_var = 4294967296
    calibTemp = self.uint8_int(calib[9])
    self.par_p3 = calibTemp/temp_var
    # Print(calibTemp)
    
    temp_var = 137438953472
    calibTemp = self.uint8_int(calib[10])
    self.par_p4 = calibTemp/temp_var
    # Print(calibTemp)
    
    temp_var = 0.125
    self.par_p5 = ((calib[12]<<8)|calib[11])/temp_var
    # Print((calib[12]<<8)|calib[11])
    
    temp_var = 64
    self.par_p6 = ((calib[14]<<8)|calib[13])/temp_var
    # Print((calib[14]<<8)|calib[13])
    
    temp_var = 256
    calibTemp = self.uint8_int(calib[15])
    self.par_p7 = calibTemp/temp_var
    # Print(calibTemp)
    
    temp_var = 32768
    calibTemp = self.uint8_int(calib[16])
    self.par_p8 = calibTemp/temp_var
    # Print(calibTemp)
    
    temp_var = 281474976710656
    self.par_p9 = ((calib[18]<<8)|calib[17])/temp_var
    # Print((calib[18]<<8)|calib[17])
    
    temp_var = 281474976710656
    calibTemp = self.uint8_int(calib[19])
    self.par_p10 = (calibTemp)/temp_var
    # Print(calibTemp)
    
    temp_var = 36893488147419103232
    calibTemp = self.uint8_int(calib[20])
    self.par_p11 = (calibTemp)/temp_var 
    # Print(calibTemp)
    
  def set_config(self):
    settings_sel = 2|4|16|32|128
    self.bmp3_set_sensor_settings(settings_sel) # Set sensor
    self.op_mode = 0x03
    self.write_power_mode()
    
  def bmp3_set_sensor_settings(self,settings_sel):
  #set_pwr_ctrl_settings
    reg_data = self.bmp3_get_regs(0x1b,1)[0]
    if(settings_sel & 2):
      reg_data = (reg_data&~(0x01))|(0x01&0x01)
    if(settings_sel & 4):
      reg_data = (reg_data&~(0x02))|((0x01<<0x01)&0x02)
    data = bytearray(1)
    data[0] = reg_data
    self.bmp3_set_regs(0x1b,data)
    
  def write_power_mode(self):
    op_mode_reg_val = self.bmp3_get_regs(0x1b,1)[0]
    op_mode_reg_val = (op_mode_reg_val&~(0x30))|((self.op_mode<<0x04)&0x30)
    data = bytearray(1)
    data[0] = op_mode_reg_val
    self.bmp3_set_regs(0x1b,data)
    
    
  def readTemperature(self):
    return round(self.bmp3_get_sensor_data(2),2)
    
  def readPressure(self):
    return round(self.bmp3_get_sensor_data(1),2)

  def bmp3_get_sensor_data(self,sensor_comp):
    rslt = self.bmp3_get_regs(0x04,6)
    #parse_sensor_data
    xlsb = rslt[0]
    lsb = rslt[1] << 8
    msb = rslt[2] << 16
    uncomp_pressure = msb|lsb|xlsb
    xlsb = rslt[3]
    lsb = rslt[4] << 8
    msb = rslt[5] << 16
    uncomp_temperature = msb|lsb|xlsb
    value = self.compensate_data(sensor_comp,uncomp_pressure,uncomp_temperature)
    return value
  
  def compensate_data(self,sensor_comp,uncomp_pressure,uncomp_temperature):
    if(sensor_comp & 0x03):
      value = self.compensate_temperature(uncomp_temperature)
    if(sensor_comp & 0x01):
      value = self.compensate_pressure(uncomp_pressure,value)
    return value
    
  def compensate_temperature(self,uncomp_temperature):
    uncomp_temp = uncomp_temperature
    partial_data1 = (uncomp_temp - self.par_t1)
    partial_data2 = (partial_data1 * self.par_t2)
    comp_temp = partial_data2 + (partial_data1 * partial_data1)*self.par_t3
    return comp_temp
    
  def compensate_pressure(self,uncomp_pressure,t_lin):
    partial_data1 = self.par_p6 * t_lin
    partial_data2 = self.par_p7 * pow(t_lin, 2)
    partial_data3 = self.par_p8 * pow(t_lin, 3)
    partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3
    partial_data1 = self.par_p2 * t_lin
    partial_data2 = self.par_p3 * pow(t_lin, 2)
    partial_data3 = self.par_p4 * pow(t_lin, 3)
    partial_out2 = uncomp_pressure *(self.par_p1-0.000145 +partial_data1+ partial_data2 + partial_data3)
    partial_data1 = pow(uncomp_pressure, 2)
    partial_data2 = self.par_p9 + self.par_p10 * t_lin
    partial_data3 = partial_data1 * partial_data2
    partial_data4 = partial_data3 + pow(uncomp_pressure, 3) * self.par_p11
    comp_press = partial_out1 + partial_out2 + partial_data4
    return comp_press;
    
  def readCalibratedAltitude(self,seaLevel):
    pressure = self.readPressure()
    return round((1.0 - pow(pressure / seaLevel, 0.190284)) * 287.15 / 0.0065,2)

  def readSeaLevel(self, altitude):
    pressure = self.readPressure()
    return round(pressure / pow(1.0 - (altitude / 44330.0), 5.255),2)

  def readAltitude(self):
    pressure = self.readPressure()
    return round((1.0 - pow(pressure / 101325, 0.190284)) * 287.15 / 0.0065,2)
  
  def INTEnable(self):
    reg_data = bytearray(1)
    reg_data[0] = 0x40;
    reg_addr = 0x19;
    self.bmp3_set_regs(reg_addr, reg_data);
  
  def INTDisable(self):
    reg_data = bytearray(1)
    reg_data[0] = 0x00;
    reg_addr = 0x19;
    self.bmp3_set_regs(reg_addr, reg_data);
class DFRobot_BMP388_SPI(DFRobot_BMP388):
  def __init__(self,spi,cs):
    self.spi = spi
    self.cs = cs    
    super(DFRobot_BMP388_SPI,self).__init__()

  def bmp3_get_regs(self,reg,len):
    regAddr = bytearray(1)
    regAddr[0] = reg|0x80
    self.cs.value(0)
    self.spi.write(regAddr)
    rslt = self.spi.read(len+1)
    self.cs.value(1)
    data = bytearray(len)
    for i in range(0,len):
      data[i] = rslt[i+1]
    return data
  
  def bmp3_set_regs(self,reg,data):
    regAddr = bytearray(1)
    regAddr[0] = reg&0x7f
    self.cs.value(0)
    self.spi.write(regAddr)
    self.spi.write(data)
    self.cs.value(1)
    
class DFRobot_BMP388_I2C(DFRobot_BMP388):
  def __init__(self,i2c):
    self.i2c = i2c
    super(DFRobot_BMP388_I2C,self).__init__()

  def bmp3_get_regs(self,reg,len):
    rslt = self.i2c.readfrom_mem(self.addr,reg,len)
    return rslt

  def bmp3_set_regs(self,reg,data):
    self.i2c.writeto_mem(self.addr,reg,data)
