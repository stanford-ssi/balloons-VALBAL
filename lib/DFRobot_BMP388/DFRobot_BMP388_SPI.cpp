/*!
 * @file DFRobot_BMP388_SPI.cpp
 * @brief DFRobot's DFRobot_BMP388
 * @n DFRobot's Temperature, Pressure and Approx altitude
 *    Supports the SPI/I2C communication with Arduino.
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yuhao](yuhao.lu@dfrobot.com)
 * @version  V1.0
 * @date  2018-5-29
 */

#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_SPI.h"
#include "bmp3_defs.h"
#include "Arduino.h"
//#include "Wire.h"
#include "SPI.h"

DFRobot_BMP388_SPI::DFRobot_BMP388_SPI(int cs):
                    DFRobot_BMP388(cs){
  
}
