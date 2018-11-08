/*!
 * @file DFRobot_BMP388.h
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
#ifndef __DFRobot_BMP388_H__
#define __DFRobot_BMP388_H__
#include "bmp3_defs.h"
#include "Arduino.h"
//#include "Wire.h"
#include "SPI.h"
#ifdef DEBUG
#define DBG() Serial.print(__LINE__);Serial.print(" : ");Serial.println(__FUNCTION__);
#else
#define DBG();
#endif


class DFRobot_BMP388
{
public:
  DFRobot_BMP388(int cs);
  int8_t begin();
  float readTemperature();
  float readPressure();
  float readCalibratedAltitude(float seaLevel);
  float readSeaLevel(float altitude);
  float readAltitude(void);
  void INTEnable();
  void INTDisable();
  
private:
  int8_t reset();
  int8_t bmp3_set_sensor_settings(uint32_t desired_settings);
  int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data);
  int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
  int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len);
  int8_t set_config();
  int8_t bmp3_set_op_mode();
  int8_t write_power_mode();
  int8_t get_calib_data();
  int8_t set_pwr_ctrl_settings(uint32_t desired_settings);
  void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data);
  int8_t compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data,struct bmp3_data *comp_data, struct bmp3_calib_data *calib_data);
  void parse_calib_data(const uint8_t *reg_data);
  double compensate_temperature(const struct bmp3_uncomp_data *uncomp_data,struct bmp3_calib_data *calib_data);
  double compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,const struct bmp3_calib_data *calib_data);
  double bmp3_pow(double base, uint8_t power);


  struct bmp3_dev dev;
  };
#endif
