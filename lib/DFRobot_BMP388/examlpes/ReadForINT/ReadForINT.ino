 /*!
  * file ReadForINT.ino
  * 
  * Connect BMP388 to IIC or SPI interface of Arduino.
  * CSB pin connect pin3/D3 if use SPI interface.
  * INT pin connect pin2/D2.
  * Download the program.
  *
  * This example describe reading data by judging the INT pin state.
  * When the data update, calling inter function,
  * after change the flag read the data.
  * @n Open serial monitor, the data could be checked. 
  *
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V0.1
  * date  2018-5-29
  */
  
#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_I2C.h"
#include "DFRobot_BMP388_SPI.h"
#include "Wire.h"
#include "SPI.h"
#include "bmp3_defs.h"

/* If BMP388_USE_I2C is 0, connect BMP388 to SPI interface of Arduino, else connect I2C interface*/
#define BMP388_USE_I2C 0

/*select CS pin*/
#ifdef __AVR__
int cs = 3;
#elif (defined ESP_PLATFORM)||(defined __ets__)
int cs = D3;
#else
  #error unknow board
#endif
/*INT pin*/
#ifdef __AVR__
int pin = 2;
#elif (defined ESP_PLATFORM)||(defined __ets__)
int pin = D2;
#else
  #error unknow board
#endif

#if BMP388_USE_I2C
DFRobot_BMP388_I2C bmp388;
#else
DFRobot_BMP388_SPI bmp388(cs);
#endif

int flag = 0;
long times = 0;
void inter(){
  flag = 1;
}

void setup(){
  /* Initialize the serial port */
  Serial.begin(9600);
  /* Initialize bmp388 */
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }
  /* connect pin4 with INT pin, set pin4 mode*/
  pinMode(pin, INPUT);
  /* config INT */
  bmp388.INTEnable();
  /*while rising read temperature and pressure*/
  attachInterrupt(digitalPinToInterrupt(pin),inter,RISING);
  times = millis();
}

void loop(){
  if(flag == 1){
  /*Read temperature and pressure*/
    float temperature = bmp388.readTemperature();
    float pressure = bmp388.readPressure();
    Serial.print("temperature : ");
    Serial.print(temperature);
    Serial.print(" C");
    Serial.print("   pressure : ");
    Serial.print(pressure);
    Serial.println(" Pa");
    flag = 0;
    delay(100);
  }
  /* After 10 seconds disable INT */
  if(millis() - times >= 10000){
    bmp388.INTDisable();
  }
}