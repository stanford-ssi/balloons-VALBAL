 /*!
  * file I2CReadTemperature.ino
  * 
  * Connect BMP388 to IIC interface of Arduino, download the program.
  * @n Open serial monitor, the temperature could be checked. 
  *
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V0.1
  * date  2018-5-29
  */
#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_I2C.h"
#include "Wire.h"
#include "SPI.h"
#include "bmp3_defs.h"

/*Create a bmp388 object to communicate with IIC.*/
DFRobot_BMP388_I2C bmp388;

void setup(){
  /*Initialize the serial port*/
  Serial.begin(9600);
  /*Initialize bmp388*/
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }
}

void loop(){
  /* Read the temperature, print data via serial port.*/
  float Temperature = bmp388.readTemperature();
  Serial.print("Temperature : ");
  Serial.print(Temperature);
  Serial.println(" C");
  delay(100);
}