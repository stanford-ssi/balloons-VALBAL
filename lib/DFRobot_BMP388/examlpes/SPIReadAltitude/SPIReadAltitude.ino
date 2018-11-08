 /*!
  * file SPIReadAltitude.ino
  * 
  * Connect BMP388 to SPI interface of Arduino and connect CSB pin to pin3 of Arduino,
  * download the program.
  *
  * Altitude is calculated based on temperature and sea level pressure.
  * The example can count an approximate altitude.
  *
  * @n pen the serial monitor, check the altitude.
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
#include "math.h"
#include "bmp3_defs.h"

/* If there is no need to calibrate altitude, comment this line */
#define CALIBRATE_Altitude

/*select CS pin*/
#ifdef __AVR__
int cs = 3;
#elif (defined ESP_PLATFORM)||(defined __ets__)
int cs = D3;
#else
  #error unknow board
#endif
/* Create a bmp388 object of SPI interface and the SPI chip selection pin is 3 */
DFRobot_BMP388_SPI bmp388(cs);

float seaLevel;

void setup(){
  /* Initialize the serial port */
  Serial.begin(9600);
  /* Initialize bmp388 */
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }
  /* You can use an accurate altitude to calibrate sea level air pressure. 
   * And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
   * In this case,525.0m is chendu accurate altitude.
   */
  delay(100);
  seaLevel = bmp388.readSeaLevel(525.0);
  Serial.print("seaLevel : ");
  Serial.print(seaLevel);
  Serial.println(" Pa");
}

void loop(){
  #ifdef CALIBRATE_Altitude
  /* Read the calibrated altitude */
  float altitude = bmp388.readCalibratedAltitude(seaLevel);
  Serial.print("calibrate Altitude : ");
  Serial.print(altitude);
  Serial.println(" m");
  #else
  /* Read the altitude */
  float altitude = bmp388.readAltitude();
  Serial.print("Altitude : ");
  Serial.print(altitude);
  Serial.println(" m");
  #endif
  delay(100);
}


