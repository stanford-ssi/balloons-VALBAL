#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <SPI.h>
#include "Config.h"

/*
Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX11120-MAX11128.pdf

When Teensy is clock:
- SCLK (clock) oscillates at 16 MHz or less
- Clock plarity (CPOL) = Clock Phase (CPHA) in uP controller registers
- SCLK should be high be default
- CS low means DIN on rising edge of clock, DOUT updated on falling edge of clock
- CS low again marks end of transmission
- minimum of 16 bits/transmission
- DOUT is 16 bits
  - If ADC mode control register has CHAN_ID = 0
    - first bit 0
    - next 12 bits data
    - last three bits 0
  - If ADC mode control register had CHAN_ID = 1
    - 4 bit channel address
    - 12 bit data (MSB first if channel is 1 in ADC control register)

When using internal Clock (need CNVST?) are we doing this?
 - clock oscillates at ~13.33MHz



 What we need:
 - Teensy drives the clock
 - Differential mode
 - Bipolar mode
*/


/* ADC MODE CONTROL */

// bit field - Table 2
typedef struct {
  uint16_t REG_CNTL: 1;
  uint16_t SCAN    : 4;
  uint16_t CHSEL   : 4;
  uint16_t RESET   : 2;
  uint16_t PM      : 2;
  uint16_t CHAN_ID : 1;
  uint16_t SWCNV   : 1;
} current_sensor_mode_control_t;

// Table 3
typedef enum {
  ADC_SCAN_NOTHING   = 0,
  ADC_SCAN_MANUAL    = 1,
  ADC_SCAN_REPEAT    = 2,
  ADC_SCAN_STD_INT   = 3,
  ADC_SCAN_STD_EXT   = 4,
  ADC_SCAN_UPP_INT   = 5,
  ADC_SCAN_UPP_EXT   = 6,
  ADC_SCAN_CUST_INT  = 7,
  ADC_SCAN_CUST_EXT  = 8,
  ADC_SCAN_SAMPLESET = 9
} current_sensor_scan_mode_t;


// Table 2
typedef enum {
  ADC_RESET_NONE = 0,
  ADC_RESET_FIFO = 1,
  ADC_RESET_REG_FIFO = 2
} current_sensor_reset_t;

// Table 5
typedef enum {
  ADC_PWR_NORMAL = 0,
  ADC_PWR_AUTO_SHUTDOWN = 1,
  ADC_PWR_AUTO_STANDBY = 2
} current_sensor_pwr_t;





/* ADC CONFIGURATION REGISTER */
enum {
  CURRENT_SENSOR_CONFIG = 0b10000,
  CURRENT_SENSOR_RANGE  = 0b10011,
  CURRENT_SENSOR_UNI    = 0b10001,
  CURRENT_SENSOR_BIP    = 0b10010,
  CURRENT_SENSOR_CUST0  = 0b10100,
  CURRENT_SENSOR_CUST1  = 0b10101
};


// bit field - Table 6
typedef struct {
  uint16_t SETUP       : 5;
  uint16_t REFSEL      : 1;
  uint16_t AVGON       : 1; // Internal Clock Mode ONLY
  uint16_t NAVG        : 2; // Internal Clock Mode ONLY
  uint16_t NSCAN       : 2; // Repeat Mode ONLY
  uint16_t SPM         : 2;
  uint16_t ECHO        : 1;
} current_sensor_config_reg_t;



/*
   RANGE REGISTER - only used to set bipolar, fully differential anolog input configurations
   UNIPOLAR/BIPOLAR REGISTER
*/

// bitfield - table 7, 10, 11
typedef struct {
  uint16_t SETUP: 5;
  uint16_t AIN_0_1:    1;
  uint16_t AIN_2_3:    1;
  uint16_t AIN_4_5:    1;
  uint16_t AIN_6_7:    1;
  uint16_t AIN_8_9:    1;
  uint16_t AIN_10_11:  1;
  uint16_t AIN_12_13:  1;
  uint16_t AIN_14_15:  1;
  uint16_t PDIF_COM:   1; // only valid for UNIPOLAR REGISTER
} current_sensor_diff_reg_t;


/* CUSTOM REGISTERS - table 12, 13 */
typedef struct {
  uint16_t SETUP: 5;
  uint16_t CHSCAN_15_7:  1;
  uint16_t CHSCAN_14_6:  1;
  uint16_t CHSCAN_13_5:  1;
  uint16_t CHSCAN_12_4:  1;
  uint16_t CHSCAN_11_3:  1;
  uint16_t CHSCAN_10_2:  1;
  uint16_t CHSCAN_9_1:   1;
  uint16_t CHSCAN_8_0:   1;
} current_sensor_custom_reg_t;

/* SampleSet Register */
typedef struct {
  uint16_t SETUP: 5;
  uint16_t SEQLEN: 8;
} current_sensor_sampleset_reg_t;

typedef enum {
  SINGLE_0 = 0,
	SINGLE_1 = 1,
	SINGLE_2 = 2,
	SINGLE_3 = 3,
	SINGLE_4 = 4,
	SINGLE_5 = 5,
	SINGLE_6 = 6,
	SINGLE_7 = 7,
	SINGLE_8 = 8,
	SINGLE_9 = 9,
	SINGLE_10 = 10,
	SINGLE_11 = 11,
	SINGLE_12 = 12,
	SINGLE_13 = 13,
	SINGLE_14 = 14,
	SINGLE_15 = 15,

  // for differential readings, only sample the even ones QUESTION
  DIFF_0_1 = 0,
  DIFF_2_3 = 2,
  DIFF_4_5 = 4,
  DIFF_6_7 = 6,
  DIFF_8_9 = 8,
  DIFF_10_11 = 10,
  DIFF_12_13 = 12,
  DIFF_14_15 = 14
} current_sensor_channel_t;

class CurrentSensor {
public:

  /**********************************  SETUP  ***********************************/
  CurrentSensor():
    current_sensor_settings(CURRENT_SENSOR_CLOCK_SPEED, MSBFIRST, SPI_MODE3) { // Mode 3 b/c "CPOL = CPHA = 1."
  }

  bool init(uint8_t chip_select_pin);

  /********************************  FUNCTIONS  *********************************/
  uint16_t set_mode_control(current_sensor_mode_control_t mode);
  uint16_t set_config_reg(current_sensor_config_reg_t config_reg);
  uint16_t set_range_reg(current_sensor_diff_reg_t range_reg);
  uint16_t set_bipolar_reg(current_sensor_diff_reg_t bipolar_reg);
  uint16_t set_unipolar_reg(current_sensor_diff_reg_t unipolar_reg);
  uint16_t set_custom_scan_0(current_sensor_custom_reg_t cust0_reg);
  uint16_t set_custom_scan_1(current_sensor_custom_reg_t cust1_reg);

  uint16_t read_data(current_sensor_channel_t channel);
  float read_voltage(current_sensor_channel_t channel);
private:
  uint16_t read_write_data(uint16_t data);
  uint8_t chip_select;
  SPISettings current_sensor_settings;


};

#endif
