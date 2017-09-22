#include <Arduino.h>

enum RadioCommandEnum {
  DATA_FRAME,
  HEARTBEAT,
  SET_CONFIG,
  SEND_BEC,
  SEND_VALBAL,
  SEND_APRS
};
typedef uint8_t RadioCommand;

enum RadioConfigEnum {
  VB_FREQUENCY,
  BEC_DIVIDER,
  APRS_DIVIDER,
  CALLSIGN_DIVIDER,
  VB_DATARATE
};
typedef uint8_t RadioConfig;

typedef struct __attribute__((__packed__)) vb_rf_message {
  RadioCommand type;
  uint8_t data[200] = {0};
} vb_rf_message;

typedef struct __attribute__((__packed__)) vb_rf_config {
  RadioConfig config;
  uint16_t data[2];
};

/* Generated with a fair dice. */
uint8_t RADIO_START_SEQUENCE[] = {204, 105, 119, 82};
uint8_t RADIO_END_SEQUENCE[] = {162, 98, 128, 161};

const int GPIO1 = 2;
const int GPIO2 = 27; // Actually 27, but that's on the back

const int VBRF_BAUD_RATE = 115200;
