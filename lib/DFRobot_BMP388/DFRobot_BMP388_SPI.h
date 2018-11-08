/*!
 * @file DFRobot_BMP388_SPI.h
 * @brief DFRobot's DFRobot_BMP388
 * @n DFRobot's Temperature, Pressure and Approx altitude
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yuhao](yuhao.lu@dfrobot.com)
 * @version  V1.0
 * @date  2018-5-29
 */
#ifndef DFRobot_BMP388_SPI_H
#define DFRobot_BMP388_SPI_H

#include "DFRobot_BMP388.h"
#include "bmp3_defs.h"
#include "Arduino.h"
//#include "Wire.h"
#include "SPI.h"

class DFRobot_BMP388_SPI : public DFRobot_BMP388
{
  public:
    DFRobot_BMP388_SPI(int cs);
};

#endif
