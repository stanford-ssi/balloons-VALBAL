/*!
 * @file DFRobot_BMP388.cpp
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
#include "bmp3_defs.h"
#include "Arduino.h"
//#include "Wire.h"
#include "SPI.h"

 void fflushthefuck(void) {
                SPI0_MCR |= SPI_MCR_CLR_RXF;
 }


int nCS;
void user_delay_ms(uint32_t num){
  delay(num);
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr,uint8_t *data, uint16_t len){
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  uint8_t i = 0;
  digitalWrite(nCS , LOW);
  SPI.transfer(reg_addr);
  for(i = 0; i < len; i++){
    SPI.transfer(*data);
    data++;
  }
  digitalWrite(nCS ,HIGH);
  return 0;
}


int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr,uint8_t *data, uint16_t len){
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  //fflushthefuck();
  int i = 0;
  digitalWrite(nCS , LOW);
  SPI.transfer(reg_addr);
  for(i = 0; i < len; i++){
    data[i] = SPI.transfer(0x00);
  }
  digitalWrite(nCS ,HIGH);
  return 0;
}


DFRobot_BMP388::DFRobot_BMP388(int cs){
  nCS = cs;
  dev.dev_id = 0;
  dev.intf = BMP3_SPI_INTF;
  dev.read = user_spi_read;
  dev.write = user_spi_write;
  dev.delay_ms = user_delay_ms;
  pinMode(nCS,OUTPUT);
  SPI.begin();
}


int8_t DFRobot_BMP388::set_config()
{
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    dev.settings.press_en = BMP3_ENABLE;
    dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    dev.settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    dev.settings.odr_filter.odr = BMP3_ODR_200_HZ;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel);

    /* Set the power mode to normal mode */
    dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode();

    return rslt;
}


float DFRobot_BMP388::readTemperature(){
  uint8_t sensor_comp;
  sensor_comp = BMP3_TEMP;
  struct bmp3_data data;
  bmp3_get_sensor_data(sensor_comp, &data);
  return data.temperature;
}

float DFRobot_BMP388::readPressure(){
  uint8_t sensor_comp;
  sensor_comp = BMP3_PRESS;
  struct bmp3_data data;
  bmp3_get_sensor_data(sensor_comp, &data);
  return data.pressure;
}

int8_t DFRobot_BMP388::begin()
{
  int8_t rslt;
  uint8_t chip_id = 0;
  /* Read the chip-id of bmp3 sensor */
  DBG();
  rslt = bmp3_get_regs(BMP3_CHIP_ID_ADDR, &chip_id, 1);
  /* Proceed if everything is fine until now */
  Serial.println(chip_id);
  if (rslt == BMP3_OK) {
		Serial.println("BMP3 okkk");
		Serial.println(chip_id);
    /* Check for chip id validity */
    if (chip_id == BMP3_CHIP_ID || true) {
		Serial.println("BMP3 chip id");
      dev.chip_id = chip_id;
      /* Reset the sensor */
      rslt = reset();
      if (rslt == BMP3_OK) {
		Serial.println("BMP3 OK");
        /* Read the calibration data */
        rslt = get_calib_data();
      }
    } else {
		Serial.println("BMP3 e dev not found");
      rslt = BMP3_E_DEV_NOT_FOUND;
      return rslt;
    }
  } else {
		Serial.println("not found");
	}

  set_config();
  return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t DFRobot_BMP388::bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  uint16_t temp_len = len +1;
  uint16_t i;
  uint8_t temp_buff[len + 1];
  //-----SPI-----
  if (dev.intf == BMP3_SPI_INTF) {
    reg_addr = reg_addr | 0x80;
    dev.read(dev.dev_id, reg_addr, temp_buff, temp_len);
    for (i = 0; i < len; i++){
      reg_data[i] = temp_buff[i + 1];
    }
  }
  //-----I2C-----
  else if(dev.intf == BMP3_I2C_INTF){
    dev.read(dev.dev_id, reg_addr, reg_data, len);
  }
  return 0;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t DFRobot_BMP388::bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
  int8_t rslt;
  uint8_t temp_buff[len * 2];
  uint16_t temp_len;
  uint8_t reg_addr_cnt;

  // Check for arguments validity
  if ((reg_addr != NULL) && (reg_data != NULL) && (len != 0)) {
    temp_buff[0] = reg_data[0];
    if (dev.intf == BMP3_SPI_INTF) {
      for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
        reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
    }
    rslt = dev.write(dev.dev_id, reg_addr[0], temp_buff, len);
  }
  return rslt;
}

/*!
 * @brief This API sets the power control(pressure enable and
 * temperature enable), over sampling, odr and filter
 * settings in the sensor.
 */
int8_t DFRobot_BMP388::bmp3_set_sensor_settings(uint32_t desired_settings)
{
  int8_t rslt;
  if (POWER_CNTL&desired_settings) {
    /* Set the power control settings */
    rslt = set_pwr_ctrl_settings(desired_settings);
  }
  return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t DFRobot_BMP388::reset()
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_CMD_ADDR;
  /* 0xB6 is the soft reset command */
  uint8_t soft_rst_cmd = 0xB6;
  uint8_t cmd_rdy_status;
  uint8_t cmd_err_status;

  /* Check for command ready status */
  DBG();
  rslt = bmp3_get_regs(BMP3_SENS_STATUS_REG_ADDR, &cmd_rdy_status, 1);
  /* Device is ready to accept new command */
  if ((cmd_rdy_status & BMP3_CMD_RDY) && (rslt == BMP3_OK)) {
    /* Write the soft reset command in the sensor */
    rslt = bmp3_set_regs(&reg_addr, &soft_rst_cmd, 1);
    /* Proceed if everything is fine until now */
    DBG();
    if (rslt == BMP3_OK) {
      /* Wait for 2 ms */
      dev.delay_ms(2);
      /* Read for command error status */
      rslt = bmp3_get_regs(BMP3_ERR_REG_ADDR, &cmd_err_status, 1);
      /* check for command error status */
      if ((cmd_err_status & BMP3_CMD_ERR) || (rslt != BMP3_OK)) {
        /* Command not written hence return
           error */
        rslt = BMP3_E_CMD_EXEC_FAILED;
      }
    }
  } else {
    rslt = BMP3_E_CMD_EXEC_FAILED;
  }
  return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t DFRobot_BMP388::bmp3_set_op_mode()
{
  int8_t rslt;
  rslt = write_power_mode();
  return rslt;
}


/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 */
int8_t DFRobot_BMP388::bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data)
{
  int8_t rslt;
  /* Array to store the pressure and temperature data read from
  the sensor */
  uint8_t reg_data[BMP3_P_T_DATA_LEN] = {0};
  struct bmp3_uncomp_data uncomp_data = {0};
  if ((comp_data != NULL)) {
    /* Read the pressure and temperature data from the sensor */
    DBG();
    rslt = bmp3_get_regs(BMP3_DATA_ADDR, reg_data, BMP3_P_T_DATA_LEN);
    if (rslt == BMP3_OK) {
      /* Parse the read data from the sensor */
      parse_sensor_data(reg_data, &uncomp_data);
      /* Compensate the pressure/temperature/both data read
         from the sensor */
      rslt = compensate_data(sensor_comp, &uncomp_data, comp_data, &dev.calib_data);
    }
  } else {
    rslt = BMP3_E_NULL_PTR;
  }

  return rslt;
}


/*!
 * @brief This internal API writes the power mode in the sensor.
 */
int8_t DFRobot_BMP388::write_power_mode()
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_PWR_CTRL_ADDR;
  uint8_t op_mode = dev.settings.op_mode;
  /* Temporary variable to store the value read from op-mode register */
  uint8_t op_mode_reg_val;

  /* Read the power mode register */
  DBG();
  rslt = bmp3_get_regs(reg_addr, &op_mode_reg_val, 1);
  /* Set the power mode */
  if (rslt == BMP3_OK) {
    op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);
    /* Write the power mode in the register */
    rslt = bmp3_set_regs(&reg_addr, &op_mode_reg_val, 1);
  }

  return rslt;
}



/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it then compensates it and store in the device structure.
 */
int8_t DFRobot_BMP388::get_calib_data()
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_CALIB_DATA_ADDR;
  /* Array to store calibration data */
  uint8_t calib_data[BMP3_CALIB_DATA_LEN] = {0};

  /* Read the calibration data from the sensor */
  DBG();
  rslt = bmp3_get_regs(reg_addr, calib_data, BMP3_CALIB_DATA_LEN);
  /* Parse calibration data and store it in device structure */
  parse_calib_data(calib_data);

  return rslt;
}

/*!
 * @brief This API sets the pressure enable and temperature enable
 * settings of the sensor.
 */
int8_t DFRobot_BMP388::set_pwr_ctrl_settings(uint32_t desired_settings)
{
  int8_t rslt;
  uint8_t reg_addr = BMP3_PWR_CTRL_ADDR;
  uint8_t reg_data;
  DBG();
  rslt = bmp3_get_regs(reg_addr, &reg_data, 1);

  if (rslt == BMP3_OK) {
    if (desired_settings & BMP3_PRESS_EN_SEL) {
      /* Set the pressure enable settings in the
      register variable */
      reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN, dev.settings.press_en);
    }
    if (desired_settings & BMP3_TEMP_EN_SEL) {
      /* Set the temperature enable settings in the
      register variable */
      reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, dev.settings.temp_en);
    }
    /* Write the power control settings in the register */
    rslt = bmp3_set_regs(&reg_addr, &reg_data, 1);
  }

  return rslt;
}

/*!
 *  @brief This internal API is used to parse the pressure or temperature or
 *  both the data and store it in the bmp3_uncomp_data structure instance.
 */
void DFRobot_BMP388::parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data)
{
  /* Temporary variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  /* Store the parsed register values for pressure data */
  data_xlsb = (uint32_t)reg_data[0];
  data_lsb = (uint32_t)reg_data[1] << 8;
  data_msb = (uint32_t)reg_data[2] << 16;
  uncomp_data->pressure = data_msb | data_lsb | data_xlsb;
  //Serial.println(uncomp_data->pressure);
  /* Store the parsed register values for temperature data */
  data_xlsb = (uint32_t)reg_data[3];
  data_lsb = (uint32_t)reg_data[4] << 8;
  data_msb = (uint32_t)reg_data[5] << 16;
  uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
  //Serial.println(uncomp_data->temperature);
}

/*!
 * @brief This internal API is used to compensate the pressure or temperature
 * or both the data according to the component selected by the user.
 */
int8_t DFRobot_BMP388::compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data,
             struct bmp3_data *comp_data, struct bmp3_calib_data *calib_data)
{
  int8_t rslt = BMP3_OK;

  if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
    /* If pressure or temperature component is selected */
    if (sensor_comp & (BMP3_PRESS | BMP3_TEMP)) {
      /* Compensate the temperature data */
      comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
    }
    if (sensor_comp & BMP3_PRESS) {
      /* Compensate the pressure data */
      comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
    }
  } else {
    rslt = BMP3_E_NULL_PTR;
  }

  return rslt;
}

/*!
 *  @brief This internal API is used to parse the calibration data, compensates
 *  it and store it in device structure
 */
void DFRobot_BMP388::parse_calib_data(const uint8_t *reg_data)
{
  /* Temporary variable to store the aligned trim data */
  struct bmp3_reg_calib_data *reg_calib_data = &dev.calib_data.reg_calib_data;
  struct bmp3_quantized_calib_data *quantized_calib_data = &dev.calib_data.quantized_calib_data;
  /* Temporary variable */
  double temp_var;

  /* 1 / 2^8 */
  temp_var = 0.00390625f;
  reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
  quantized_calib_data->par_t1 = ((double)reg_calib_data->par_t1 / temp_var);

  reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
  temp_var = 1073741824.0f;
  quantized_calib_data->par_t2 = ((double)reg_calib_data->par_t2 / temp_var);

  reg_calib_data->par_t3 = (int8_t)reg_data[4];
  temp_var = 281474976710656.0f;
  quantized_calib_data->par_t3 = ((double)reg_calib_data->par_t3 / temp_var);

  reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);

  temp_var = 1048576.0f;
  quantized_calib_data->par_p1 = ((double)(reg_calib_data->par_p1 - (16384)) / temp_var);

  reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
  temp_var = 536870912.0f;
  quantized_calib_data->par_p2 = ((double)(reg_calib_data->par_p2 - (16384)) / temp_var);

  reg_calib_data->par_p3 = (int8_t)reg_data[9];
  temp_var = 4294967296.0f;
  quantized_calib_data->par_p3 = ((double)reg_calib_data->par_p3 / temp_var);

  reg_calib_data->par_p4 = (int8_t)reg_data[10];
  temp_var = 137438953472.0f;
  quantized_calib_data->par_p4 = ((double)reg_calib_data->par_p4 / temp_var);

  reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
  /* 1 / 2^3 */
  temp_var = 0.125f;
  quantized_calib_data->par_p5 = ((double)reg_calib_data->par_p5 / temp_var);

  reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14],  reg_data[13]);
  temp_var = 64.0f;
  quantized_calib_data->par_p6 = ((double)reg_calib_data->par_p6 / temp_var);

  reg_calib_data->par_p7 = (int8_t)reg_data[15];
  temp_var = 256.0f;
  quantized_calib_data->par_p7 = ((double)reg_calib_data->par_p7 / temp_var);

  reg_calib_data->par_p8 = (int8_t)reg_data[16];
  temp_var = 32768.0f;
  quantized_calib_data->par_p8 = ((double)reg_calib_data->par_p8 / temp_var);

  reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
  temp_var = 281474976710656.0f;
  quantized_calib_data->par_p9 = ((double)reg_calib_data->par_p9 / temp_var);

  reg_calib_data->par_p10 = (int8_t)reg_data[19];
  temp_var = 281474976710656.0f;
  quantized_calib_data->par_p10 = ((double)reg_calib_data->par_p10 / temp_var);

  reg_calib_data->par_p11 = (int8_t)reg_data[20];
  temp_var = 36893488147419103232.0f;
  quantized_calib_data->par_p11 = ((double)reg_calib_data->par_p11 / temp_var);
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
double DFRobot_BMP388::compensate_temperature(const struct bmp3_uncomp_data *uncomp_data,
            struct bmp3_calib_data *calib_data)
{
  uint32_t uncomp_temp = uncomp_data->temperature;
  double partial_data1;
  double partial_data2;

  partial_data1 = (double)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
  partial_data2 = (double)(partial_data1 * calib_data->quantized_calib_data.par_t2);
  /* Update the compensated temperature in calib structure since this is
     needed for pressure calculation */
  calib_data->quantized_calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1)
              * calib_data->quantized_calib_data.par_t3;

  /* Return compensated temperature */
  return calib_data->quantized_calib_data.t_lin;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
double DFRobot_BMP388::compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,
          const struct bmp3_calib_data *calib_data)
{
  const struct bmp3_quantized_calib_data *quantized_calib_data = &calib_data->quantized_calib_data;
  /* Variable to store the compensated pressure */
  double comp_press;
  /* Temporary variables used for compensation */
  double partial_data1;
  double partial_data2;
  double partial_data3;
  double partial_data4;
  double partial_out1;
  double partial_out2;

  partial_data1 = quantized_calib_data->par_p6 * quantized_calib_data->t_lin;
  partial_data2 = quantized_calib_data->par_p7 * bmp3_pow(quantized_calib_data->t_lin, 2);
  partial_data3 = quantized_calib_data->par_p8 * bmp3_pow(quantized_calib_data->t_lin, 3);
  partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;


  partial_data1 = quantized_calib_data->par_p2 * quantized_calib_data->t_lin;
  partial_data2 = quantized_calib_data->par_p3 * bmp3_pow(quantized_calib_data->t_lin, 2);
  partial_data3 = quantized_calib_data->par_p4 * bmp3_pow(quantized_calib_data->t_lin, 3);
  partial_out2 = uncomp_data->pressure *
      (quantized_calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);



  partial_data1 = bmp3_pow((double)uncomp_data->pressure, 2);
  partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + bmp3_pow((double)uncomp_data->pressure, 3) * quantized_calib_data->par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;
  return comp_press;
}

/*!
 * @brief This internal API is used to calculate the power functionality for
 * double precision floating point values.
 */
double DFRobot_BMP388::bmp3_pow(double base, uint8_t power)
{
  double pow_output = 1;

  while (power != 0) {
    pow_output = base * pow_output;
    power--;
  }

  return pow_output;
}

float DFRobot_BMP388::readCalibratedAltitude(float seaLevel)
{
  float pressure = readPressure();
  return (1.0 - pow((float)pressure / seaLevel, 0.190284)) * 287.15 / 0.0065;
}

float DFRobot_BMP388::readSeaLevel(float altitude)
{
  float pressure = readPressure();
  return (pressure / pow(1.0 - (altitude / 44330.0), 5.255));
}

float DFRobot_BMP388::readAltitude(void)
{
  float pressure = readPressure();
  return (1.0 - pow(pressure / 101325, 0.190284)) * 287.15 / 0.0065;
}

void DFRobot_BMP388::INTEnable(){
  uint8_t reg_data = 0x40;
  uint8_t reg_addr = BMP3_INT_CTRL_ADDR;
  bmp3_set_regs(&reg_addr, &reg_data, 1);
}

void DFRobot_BMP388::INTDisable(){
  uint8_t reg_data = 0x00;
  uint8_t reg_addr = BMP3_INT_CTRL_ADDR;
  bmp3_set_regs(&reg_addr, &reg_data, 1);
}
