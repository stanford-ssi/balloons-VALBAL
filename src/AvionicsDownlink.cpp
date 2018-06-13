#include "Avionics.h"

/*
 * Function: compressVariable
 * -------------------
 * This function compresses a single variable into a scaled digital bitmask.
 */
int16_t Avionics::compressVariable(float var, float minimum, float maximum, int16_t resolution, int16_t length) {
  if (resolution <= 0) return -1;
  if (var < minimum) var = minimum;
  if (var > maximum) var = maximum;
  int32_t adc = round( (pow(2, resolution) - 1) * (var - minimum) / (maximum - minimum));
  int16_t byteIndex = length / 8;
  int16_t bitIndex = 7 - (length % 8);
  for (int16_t i = resolution - 1; i >= 0; i--) {
    bool bit = adc & (1 << i);
    if (bit) COMMS_BUFFER[byteIndex] |= (1 << bitIndex);
    bitIndex -= 1;
    if (bitIndex < 0) {
      bitIndex = 7;
      byteIndex++;
    }
  }
  return resolution;
}

/*
 * Function: compressData
 * -------------------
 * This function compresses the data frame into a bit stream.
 * The total bitstream cannot exceed 400 bytes.
 */
int16_t Avionics::compressData() {
  int16_t lengthBits  = 0;
  int16_t lengthBytes = 0;
  for(uint16_t i = 0; i < COMMS_BUFFER_SIZE; i++) COMMS_BUFFER[i] = 0;
  lengthBits += compressVariable(data.TIME / 1000,                           0,    3000000, 20, lengthBits); // time
  lengthBits += compressVariable(data.LAT_GPS,                              -90,   90,      21, lengthBits); // latitude
  lengthBits += compressVariable(data.LONG_GPS,                             -180,  180,     22, lengthBits); // longitude
  lengthBits += compressVariable(data.ALTITUDE_BAROMETER,                   -2000, 40000,   16, lengthBits); // altitude_barometer
  lengthBits += compressVariable(data.ALTITUDE_GPS,                         -2000, 40000,   14, lengthBits);
  lengthBits += compressVariable(data.ASCENT_RATE,                          -10,   10,      11, lengthBits);
  lengthBits += compressVariable(data.CURRENT_CONTROLLER_INDEX,             0,    3,        2,  lengthBits);
  lengthBits += compressVariable(data.VALVE_STATE,                           0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_STATE,                         0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.VALVE_QUEUE / 1000,                    0,    1023,    10, lengthBits);
  lengthBits += compressVariable(data.BALLAST_QUEUE / 1000,                  0,    1023,    10, lengthBits);
  lengthBits += compressVariable(data.VALVE_TIME_TOTAL / 1000,               0,    16383,   13, lengthBits); // valve time total
  lengthBits += compressVariable(data.BALLAST_TIME_TOTAL / 1000,             0,    16383,   13, lengthBits); // ballast time total
  lengthBits += compressVariable(data.BALLAST_DIRECTION,                     0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.VALVE_NUM_ACTIONS,                     0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_NUM_ACTIONS,                   0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.VALVE_NUM_ATTEMPTS,                    0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_NUM_ATTEMPTS,                  0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.BALLAST_NUM_OVERCURRENTS,              0,    63,      6,  lengthBits);
  lengthBits += compressVariable(data.CUTDOWN_STATE,                         0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.MAX_CURRENT_CHARGING_LIMIT,            0,    3,       2,  lengthBits);
  lengthBits += compressVariable(data.SYSTEM_POWER_STATE,                    0,    3,       2,  lengthBits);
  lengthBits += compressVariable(data.TEMP_INT,                             -85,   65,      9,  lengthBits);
  lengthBits += compressVariable(data.JOULES_TOTAL,                          0,    1572863, 18, lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_PRIMARY,                       0,    6,       9,  lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_PRIMARY_MIN,                   0,    6,       9,  lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_SUPERCAP_AVG,                  0,    6,       9,  lengthBits);
  lengthBits += compressVariable(data.VOLTAGE_SUPERCAP_MIN,                  0,    6,       9,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_TOTAL_AVG,                     0,    4095,    12, lengthBits);
  lengthBits += compressVariable(data.CURRENT_TOTAL_MIN,                     0,    4095,    12, lengthBits);
  lengthBits += compressVariable(data.CURRENT_TOTAL_MAX,                     0,    4095,    12, lengthBits);
  lengthBits += compressVariable(data.CURRENT_RB_AVG,                        0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_RB_MAX,                        0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_VALVE_AVG,               0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_VALVE_MAX,               0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_BALLAST_AVG,             0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_MOTOR_BALLAST_MAX,             0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_PAYLOAD_AVG,                   0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.CURRENT_PAYLOAD_MAX,                   0,    1023,    8,  lengthBits);
  lengthBits += compressVariable(data.TEMP_EXT,                             -100,  30,      8,  lengthBits);
  lengthBits += compressVariable(data.LOOP_TIME_MAX,                         0,    10239,   10, lengthBits);
  lengthBits += compressVariable(data.RB_SENT_COMMS,                         0,    8191,    13, lengthBits);
  lengthBits += compressVariable(data.RB_RESTARTS,                           0,    31,      5,  lengthBits);
  lengthBits += compressVariable(data.RESISTOR_MODE,                         0,    4,       3,  lengthBits);
  lengthBits += compressVariable(data.MANUAL_MODE,                           0,    1,       1,  lengthBits);
  lengthBits += compressVariable(data.REPORT_MODE,                           0,    2,       2,  lengthBits);
  lengthBits += compressVariable(data.SHOULD_REPORT,                         0,    1,       1,  lengthBits);
  if (data.SHOULD_REPORT || data.REPORT_MODE != 0) {
    lengthBits += compressVariable(data.POWER_STATE_LED,                     0,    1,       1,  lengthBits); // LED Power state
    lengthBits += compressVariable(data.POWER_STATE_RB,                      0,    1,       1,  lengthBits); // RB Power State
    lengthBits += compressVariable(data.POWER_STATE_GPS,                     0,    1,       1,  lengthBits); // GPS Power State
    lengthBits += compressVariable(data.POWER_STATE_PAYLOAD,                 0,    1,       1,  lengthBits); // Payload Power State
    lengthBits += compressVariable(data.NUM_SATS_GPS,                        0,    15,      3,  lengthBits);
    lengthBits += compressVariable(data.INCENTIVE_NOISE,                     0,    4,       8,  lengthBits);
    lengthBits += compressVariable(data.VALVE_ALT_LAST,                     -2000, 50000,   11, lengthBits); // Altitude During Last Venting Event
    lengthBits += compressVariable(data.BALLAST_ALT_LAST,                   -2000, 50000,   11, lengthBits); // Altitude During Last Ballast Event
    lengthBits += compressVariable(data.DEBUG_STATE,                         0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.FORCE_VALVE,                         0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.FORCE_BALLAST,                       0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_1_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_2_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_3_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.BMP_4_ENABLE,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(log2(data.BMP_1_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_1_logrejections
    lengthBits += compressVariable(log2(data.BMP_2_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_2_logrejections
    lengthBits += compressVariable(log2(data.BMP_3_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_3_logrejections
    lengthBits += compressVariable(log2(data.BMP_4_REJECTIONS + 1),          0,    6,       4,  lengthBits); // sensor_4_logrejections
    lengthBits += compressVariable(data.ACTION / 1000,              -1023, 1023,     7,  lengthBits);
    lengthBits += compressVariable(data.VALVE_INCENTIVE,            -50,   10,       12, lengthBits);
    lengthBits += compressVariable(data.BALLAST_INCENTIVE,          -50,   10,       12, lengthBits);
    lengthBits += compressVariable(data.RE_ARM_CONSTANT,             0,    4,        8,  lengthBits);
    lengthBits += compressVariable(data.VALVE_ALT_LAST,             -2000, 50000,    11, lengthBits);
    lengthBits += compressVariable(data.BALLAST_ALT_LAST,           -2000, 50000,    11, lengthBits);
    lengthBits += compressVariable(data.SPAG_STATE.effort*1000,            -2, 2, 12, lengthBits);
    lengthBits += compressVariable(data.SPAG2_STATE.effort*1000,           -2, 2, 12, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.v,           -10,     10,   11, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.fused_v,     -10,     10,   11, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.effort,                  -2,       2,  11, lengthBits);
    lengthBits += compressVariable(data.LAS_STATE.v_cmd,                 -10,      10,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[2]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[3]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[4]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[5]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[6]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.ACTION_TIME_TOTALS[7]/1000,        0,     600,   8, lengthBits);
    lengthBits += compressVariable(data.RB_HEAT_DUTY,        0,     255,   8, lengthBits);
    lengthBits += compressVariable(data.HEATER_CONSTANTS.temp_thresh,        -100,     100,   8, lengthBits);
    lengthBits += compressVariable(data.HEATER_CONSTANTS.temp_gain,        0,     1,   8, lengthBits);
    lengthBits += compressVariable(data.HEATER_CONSTANTS.comm_gain,        0,     4,   4, lengthBits);
    lengthBits += compressVariable(data.HEATER_CONSTANTS.cap_gain,        0,     4,   4, lengthBits);
    lengthBits += compressVariable(data.HEATER_CONSTANTS.cap_nominal,        0,     5,   8, lengthBits);
    lengthBits += compressVariable(data.HEATER_CONSTANTS.max_duty,        0,     256,   8, lengthBits);
    lengthBits += compressVariable(data.IN_CUBA,                        0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.CUBA_TIMEOUT,                        0,    4000000,       10,  lengthBits);
    lengthBits += compressVariable(data.TIMED_CUTDOWN_ENABLE,            0,    1,       1,  lengthBits);
    lengthBits += compressVariable(data.GEOFENCED_CUTDOWN_ENABLE,            0,    1,       1,  lengthBits);
  }
  if (data.SHOULD_REPORT || data.REPORT_MODE == 2) {
    lengthBits += compressVariable(data.RB_INTERVAL / 1000,                  0,    1023,    10, lengthBits); // RB communication interval
    lengthBits += compressVariable(data.GPS_INTERVAL / 1000,                 0,    1023,    10, lengthBits); // GPS communication interval
    lengthBits += compressVariable(data.PRESS_BASELINE,                      0,    131071,  17, lengthBits); // Pressure baseline
    lengthBits += compressVariable(data.INCENTIVE_THRESHOLD,                 0,    4,       3,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_ARM_ALT,                    -2000, 40000,   16, lengthBits); // Ballast Arming Altitude
    lengthBits += compressVariable(data.BALLAST_REVERSE_INTERVAL / 1000,     0,    1599,    4,  lengthBits); // Ballast reverse interval
    lengthBits += compressVariable(data.BALLAST_STALL_CURRENT,               0,    511,     4,  lengthBits);
    lengthBits += compressVariable(data.VALVE_OPENING_DURATION / 1000,       0,    10,      5,  lengthBits);
    lengthBits += compressVariable(data.VALVE_CLOSING_DURATION / 1000,       0,    10,      5,  lengthBits);
    lengthBits += compressVariable(data.VALVE_SETPOINT,                     -2000, 50000,   11, lengthBits);
    lengthBits += compressVariable(data.VALVE_VENT_DURATION / 1000,          0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.VALVE_FORCE_DURATION / 1000,         0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.VALVE_VELOCITY_CONSTANT,             0,    5,       8,  lengthBits); // Valve Speed Constant
    lengthBits += compressVariable(1.0 / data.VALVE_ALTITUDE_DIFF_CONSTANT,  0,    4095,    8,  lengthBits); // Valve Altitude Difference Constant
    lengthBits += compressVariable(1.0 / data.VALVE_LAST_ACTION_CONSTANT,    0,    4095,    8,  lengthBits); // Valve last action constant
    lengthBits += compressVariable(data.BALLAST_SETPOINT,                   -2000, 50000,   11, lengthBits);
    lengthBits += compressVariable(data.BALLAST_DROP_DURATION / 1000,        0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_FORCE_DURATION / 1000,       0,    1023,    6,  lengthBits);
    lengthBits += compressVariable(data.BALLAST_VELOCITY_CONSTANT,           0,    5,       8,  lengthBits); // Ballast Speed Constant
    lengthBits += compressVariable(1.0 / data.BALLAST_ALTITUDE_DIFF_CONSTANT,0,    4095,    8,  lengthBits); // Ballast Altitude Difference Constant
    lengthBits += compressVariable(1.0 / data.BALLAST_LAST_ACTION_CONSTANT,  0,    4095,    8,  lengthBits); // Ballast last action constant
    // spaghetti readback
    lengthBits += compressVariable(data.SPAG_CONSTANTS.k,                      0,      2,    6,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.b_dldt*1000,            0,      100,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.v_dldt*1000,            0,      100,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.rate_min*1000,          0,      0.2,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.rate_max*1000,          0,      2,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.b_tmin,                 0,      31,    5,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.v_tmin,                 0,      31,    5,  lengthBits);
    lengthBits += compressVariable(data.SPAG_CONSTANTS.h_cmd,                  -2000,    40000,    11,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.k,                     0,      2,    6,      lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.b_dldt*1000,           0,      100,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.v_dldt*1000,           0,      100,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.rate_min*1000,         0,      0.2,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.rate_max*1000,         0,      2,    8,    lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.b_tmin,                0,      31,    5,   lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.v_tmin,                0,      31,    5,   lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.h_cmd,                 -2000,    40000,    11,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.v_ss_error_thresh,     0,      3000,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.b_ss_error_thresh,     0,      3000,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.ascent_rate_thresh,    0,      10,    8,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.kfuse,                 0,      30,    6,  lengthBits);
    lengthBits += compressVariable(data.SPAG2_CONSTANTS.kfuse_v,               0,      1,    4,  lengthBits);

    lengthBits += compressVariable(data.LAS_CONSTANTS.k_v,                   0,      .01,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.k_h,                   0,      .01,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.b_dldt*1000,           0,      100,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_dldt_a*1000,           0,      100,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_dldt_b*1000,           0,      100,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.b_tmin,                0,       20,   4,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_tmin,                0,       20,   4,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.h_cmd,                 0,    20000,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.kfuse,                 0,        30,  6,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.kfuse_val,             0,        1,   4,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.ss_error_thresh,       0,     3000,   8,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.v_limit,               0,        2,   6,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.equil_h_thresh,        0,    20000,  16,  lengthBits);
    lengthBits += compressVariable(data.LAS_CONSTANTS.launch_h_thresh,       0,     2000,  12,  lengthBits);
  }
  lengthBits += 8 - (lengthBits % 8);
  lengthBytes = lengthBits / 8;
  data.SHOULD_REPORT = false;
  data.COMMS_LENGTH = lengthBytes;
  if(data.DEBUG_STATE) {
    for (int16_t i = 0; i < lengthBytes; i++) {
      uint8_t byte = COMMS_BUFFER[i];
      (byte & 0x80 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x40 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x20 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x10 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x08 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x04 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x02 ? Serial.print('1') : Serial.print('0'));
      (byte & 0x01 ? Serial.print('1') : Serial.print('0'));
    }
    Serial.print('\n');
  }
  return lengthBytes;
}
