 /*!
  * file SPIReadPressure.ino
  * 
  * Connect BMP388 to SPI interface of Arduino and connect CSB pin to pin3 of Arduino,
  * download the program.
  *
  * @n Open serial monitor, the atmospheric pressure could be checked. 
  *
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V0.1
  * date  2018-5-29
  */
#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_SPI.h"
#include "SPI.h"
#include "Wire.h"
#include "bmp3_defs.h"

/*select CS pin*/
#ifdef __AVR__
int cs = 3;
#elif (defined ESP_PLATFORM)||(defined __ets__)
int cs = D3;
#else
  #error unknow board
#endif

/*Create a bmp388 object of SPI interface and the SPI chip selection pin is 3*/
DFRobot_BMP388_SPI bmp388(cs);
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
  /*Read the atmospheric pressure, print data via serial port*/
  float Pressure = bmp388.readPressure();
  Serial.print("Pressure : ");
  Serial.print(Pressure);
  Serial.println(" Pa");
  delay(100);
}