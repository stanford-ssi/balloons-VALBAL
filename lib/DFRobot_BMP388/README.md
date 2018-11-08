# BMP388 Library for Arduino/ESP32/RaspberrayPi
This is a Library for BMP388, the function is to read temperature and pressure.

## Table of Contents

* [Summary](#summary)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


<snippet>
<content>

## Summary
* BMP388 can read temperature and pressure.
* The library supports the SPI/I2C communication.
* This library provides examples of calculating  and calibrating altitude.
* The sensor is more accurate than its predecessors, covering a wide measurement range from 300 hPa to 1250 hPa.
* This new barometric pressure sensor exhibits an attractive price-performance ratio coupled with low power consumption.

## Methods

### Arduino
```C++

#include <DFRobot_BMP388.h>
/*
 * @brief bmp388 I2C constructor
 */
DFRobot_BMP388_I2C();

/*
 * @brief bmp388 SPI constructor
 *
 * @param cs Chip selection pin of SPI
 */
DFRobot_BMP388_SPI(int cs);

/*
 * @brief Initialize bmp388, check for chip id, reset the sensor, read the calibration data and config sensor
 *
 * @return 0 if success
 */
int8_t begin();

/*
 * @brief Read temperature
 *
 * @return temperature
 */
float readTemperature();

/*
 * @brief Read pressure.
 *
 * @return Pressure
 */
float readPressure();

/*
 * @brief Read normalised atmospheric pressure of sea level
 *
 * @param altitude The local elevation
 *
 * @return Pressure of sea level
 */
float readSeaLevel(float altitude);

/*
 * @brief Read calibrated elevation.
 *
 * @param seaLevel Pressure of sea level
 *
 * @return Calibrated elevation
 */
float readCalibratedElevation(float seaLevel);

/*
 * @brief Read elevation.
 *
 * @return elevation
 */
float readElevation(void);

/*
 * @brief Enable INT.
 */
void INTEnable();

/*
 * @brief Disable INT.
 */
void INTDisable();

```

### micropython
```python

import bmp388

/*
 * @brief bmp388 I2C constructor
 *
 * @param i2c I2C object.
 */
DFRobot_BMP388_I2C(i2c)

/*
 * @brief bmp388 SPI constructor
 *
 * @param spi SPI object.
 *        cs  Chip selection pin of SPI
 */
DFRobot_BMP388_SPI(spi,cs)

/*
 * @brief Read temperature
 *
 * @return Temperature
 */
readTemperature()

/*
 * @brief Read pressure.
 *
 * @return pressure
 */
readPressure()

/*
 * @brief Read normalised atmospheric pressure of sea level
 *
 * @param altitude The local elevation
 *
 * @return Pressure of sea level
 */
readSeaLevel(altitude)

/*
 * @brief Read calibrated elevation.
 *
 * @param seaLevel Pressure of sea level
 *
 * @return Calibrated elevation
 */
readCalibratedElevation(seaLevel)

/*
 * @brief Read elevation.
 *
 * @return elevation
 */
readElevation()

/*
 * @brief Enable INT.
 */
INTEnable()

/*
 * @brief Disable INT.
 */
INTDisable()
```

### Raspberry Pi
```python

import bmp388

/*
 * @brief bmp388 I2C constructor
 */
DFRobot_BMP388_I2C()

/*
 * @brief bmp388 SPI constructor
 *
 * @param cs Chip selection pin of SPI
 */
DFRobot_BMP388_SPI(cs)

/*
 * @brief Read temperature
 *
 * @return temperature
 */
readTemperature()

/*
 * @brief Read pressure.
 *
 * @return pressure
 */
readPressure()

/*
 * @brief Read normalised atmospheric pressure of sea level
 *
 * @param altitude The local elevation
 *
 * @return Pressure of sea level
 */
readSeaLevel(altitude)

/*
 * @brief Read calibrated elevation.
 *
 * @param seaLevel Pressure of sea level
 *
 * @return Calibrated elevation
 */
readCalibratedElevation(seaLevel)

/*
 * @brief Read elevation.
 *
 * @return elevation
 */
readElevation()

/*
 * @brief Enable INT.
 */
INTEnable()

/*
 * @brief Disable INT.
 */
INTDisable()
```


## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32 |      √       |             |            | 
RaspberrayPi |      √       |             |            | Only support python3
Arduino |      √       |             |            | 

## History

- data 2018-6-2
- version V0.1


## Credits

- author [Luyuhao  <yuhao.lu@dfrobot.com>]
