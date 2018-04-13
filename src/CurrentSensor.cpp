#include "CurrentSensor.h"

bool CurrentSensor::init(uint8_t chip_select_pin) {
  Serial.println("Current Sensor init");
  // setup chip select
  chip_select = chip_select_pin;
  pinMode(chip_select_pin, OUTPUT);
  digitalWrite(chip_select_pin, HIGH);
  SPI.begin();
  delay(1);

  // reset config to defaults
  read_write_data(2<<5); // from https://github.com/emard/max1112x-test/blob/master/max1112x-test.ino

  // configure the sensor how we want to for manual mode
  //current_sensor_config_reg_t config_reg;
  config_manual_reg.SETUP = CURRENT_SENSOR_CONFIG;
  config_manual_reg.REFSEL = 0; // external differential
  config_manual_reg.AVGON = 0;
  config_manual_reg.NAVG = 0;
  config_manual_reg.NSCAN = 0;
  config_manual_reg.SPM = 0; // keep the sensor powered on all the time for now QUESTION
  config_manual_reg.ECHO = 1;
  config_manual_reg.EMPTY = 0;
  // Serial.print("Config register: ");
  // Serial.println(*(uint16_t *)&config_manual_reg);
  set_config_reg(config_manual_reg);


  // set up the sensor's configuration for repeat mode.
  // config_repeat_reg.SETUP = CURRENT_SENSOR_CONFIG;
  // config_repeat_reg.REFSEL = 0; // external differential
  // config_repeat_reg.AVGON = 1;
  // config_repeat_reg.NAVG = 3; // average 32 conversions
  // config_repeat_reg.NSCAN = 3; // return 16 results in repeat mode
  // config_repeat_reg.SPM = 0; // keep the sensor powered on all the time for now
  // config_repeat_reg.ECHO = 1;
  // config_repeat_reg.EMPTY = 0;
  // Serial.print("Config register: ");
  // Serial.println(*(uint16_t *)&config_repeat_reg);
  // set_config_reg(config_repeat_reg);


  // we have a few options, either we can scan all the channels, we can
  // preset the ones we want, or we can specifiy which one to scan at a
  // given time (manual mode). The last one sounds good
  current_sensor_mode_control_t mode_reg;
  for (int channel = 8; channel < 16; channel++) {
    // initialize all the channels (this can change to just initializing the ones we want) QUESTION
    mode_reg.REG_CNTL = 0;
    mode_reg.SCAN = ADC_SCAN_MANUAL;
    //mode_reg.SCAN = ADC_SCAN_REPEAT;
    mode_reg.CHSEL = channel;
    mode_reg.RESET = 0;
    mode_reg.PM = 0; // we can change this as we need to conserve battery (see table )
    mode_reg.CHAN_ID = 1; // so we can see the channel we're getting back from the output
    //Serial.println(*(uint16_t *)&mode_reg);
    // Serial.print("Mode reg: ");
    // Serial.println(*(uint16_t *)&mode_reg);
    delay(10);
    // Serial.println(
      set_mode_control(mode_reg);
    // );
  }

  // current_sensor_diff_reg_t range_reg;

  // set up the current sensor in differential bipolar mode
  current_sensor_diff_reg_t bipolar_reg;
  bipolar_reg.SETUP = CURRENT_SENSOR_BIP;
  bipolar_reg.AIN_0_1   = USING_CHANNEL_DIFF_0_1;
  bipolar_reg.AIN_2_3   = USING_CHANNEL_DIFF_2_3;
  bipolar_reg.AIN_4_5   = USING_CHANNEL_DIFF_4_5;
  bipolar_reg.AIN_6_7   = USING_CHANNEL_DIFF_6_7;
  bipolar_reg.AIN_8_9   = USING_CHANNEL_DIFF_8_9;
  bipolar_reg.AIN_10_11 = USING_CHANNEL_DIFF_10_11;
  bipolar_reg.AIN_12_13 = USING_CHANNEL_DIFF_12_13;
  bipolar_reg.AIN_14_15 = USING_CHANNEL_DIFF_14_15;
  // Serial.print("Bipolar register: ");
  // Serial.println(*(uint16_t *)&bipolar_reg);
  set_bipolar_reg(bipolar_reg);

  current_sensor_diff_reg_t range_reg;
  range_reg.SETUP = CURRENT_SENSOR_RANGE;
  range_reg.AIN_12_13 = 1;
  // Serial.print("Range register: ");
  // Serial.println(*(uint16_t *)&range_reg);
  set_range_reg(range_reg);


  return true;
}

uint16_t CurrentSensor::read_write_data(uint16_t data) {
  SPI.beginTransaction(current_sensor_settings);
  //digitalWrite(chip_select, HIGH);
  digitalWrite(chip_select, LOW); // falling edge triggers data i/o
  uint16_t received = SPI.transfer16(data);
  digitalWrite(chip_select, HIGH);
  //digitalWrite(chip_select, LOW);
  SPI.endTransaction();
  // Serial.print("Got ");
  // Serial.println(received);
  return received;
}



uint16_t CurrentSensor::set_mode_control(current_sensor_mode_control_t mode) {
  return read_write_data(*(uint16_t *)&mode);
}
uint16_t CurrentSensor::set_config_reg(current_sensor_config_reg_t config_reg){
  return read_write_data(*(uint16_t *)&config_reg);
}
uint16_t CurrentSensor::set_range_reg(current_sensor_diff_reg_t range_reg){
  return read_write_data(*(uint16_t *)&range_reg);
}
uint16_t CurrentSensor::set_bipolar_reg(current_sensor_diff_reg_t bipolar_reg) {
  return read_write_data(*(uint16_t *)&bipolar_reg);
}
uint16_t CurrentSensor::set_unipolar_reg(current_sensor_diff_reg_t unipolar_reg) {
  return read_write_data(*(uint16_t *)&unipolar_reg);
}
uint16_t CurrentSensor::set_custom_scan_0(current_sensor_custom_reg_t cust0_reg) {
  return read_write_data(*(uint16_t *)&cust0_reg);
}
uint16_t CurrentSensor::set_custom_scan_1(current_sensor_custom_reg_t cust1_reg) {
  return read_write_data(*(uint16_t *)&cust1_reg);
}

uint16_t CurrentSensor::read_data(current_sensor_channel_t channel) {
  current_sensor_mode_control_t mode_reg;
  mode_reg.REG_CNTL = 0;
  mode_reg.SCAN = ADC_SCAN_MANUAL;
  //mode_reg.SCAN = ADC_SCAN_REPEAT;
  mode_reg.CHSEL = channel;
  mode_reg.RESET = 0;
  mode_reg.PM = 0;
  mode_reg.CHAN_ID = 1;
  // Serial.println(*(uint16_t *)&mode_reg);
  set_mode_control(mode_reg); // first write is to select the channel
  //delay(1);
  uint16_t result = set_mode_control(mode_reg); // next write is to get the output
  //Serial.println(result);
  return result;
  // this is NOT the most efficient way to do this, but it should work
  // I'm going to talk with Sasha to figure out what exactly it is that
  // we need to do
}

float CurrentSensor::read_voltage(current_sensor_channel_t channel) {
  //return 0; // shut up current sensor
  // read the raw data over SPI
  uint16_t raw_data = read_data(channel);

  // make sure the channel was correct
  uint16_t result_channel = (raw_data & (0xf << 12)) >> 12;
  //Serial.println(raw_data);

  int16_t data = raw_data & 0xfff; // lower 12 bits in 2's complement representing voltage
  // Serial.print("ultra raw: ");
  // Serial.println(raw_data);
  // Serial.print("raw: ");
  // Serial.println(data);
  // convert raw_data to full 16 bit signed int
  data = (data * (1 << 4))/(1<<4); // move the signed bits over 4 places (to the MSB of the 16-bit int) and
                                  // then perfrom an arithmetic right shift by dividing by the same amount
                                  // Serial.print("voltage: ");
                                  // Serial.println((CURRENT_SENSOR_VREF / 2) * ( (float) data / CURRENT_SENSOR_MAX_BIP), 6);
  return (CURRENT_SENSOR_VREF / 2) * ( (float) data / CURRENT_SENSOR_MAX_BIP);


}

float CurrentSensor::average_voltage_readings(current_sensor_channel_t channel, uint32_t num_samples) {
  float current_value = 0;
  for (uint32_t i = 0; i < num_samples; i++) {
    current_value += read_voltage(channel);
  }
  return current_value / num_samples;
}
// uint16_t CurrentSensor::repeat_sample_channel(current_sensor_channel_t channel) {
//   // configure for repeat scan mode
//   // set_config_reg(config_repeat_reg);
//
//
// }
