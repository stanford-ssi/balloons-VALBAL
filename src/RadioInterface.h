#ifndef RADIOINTERFACE_H
#define RADIOINTERFACE_H

#include <Arduino.h>

typedef struct __attribute__((__packed__)) vb_rf_message {
  uint8_t data[200] = {0};
} vb_rf_message;

/* Generated with a fair dice. */
const uint8_t RADIO_START_SEQUENCE[] = {204, 105, 119, 82};
const uint8_t RADIO_END_SEQUENCE[] = {162, 98, 128, 161};

const int VBRF_BAUD_RATE = 115200;

#endif
