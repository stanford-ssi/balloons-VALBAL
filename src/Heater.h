/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2018

  Jonathan Zwiebel | jzwiebel@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Heater.h
  --------------------------
  Code to control a heater
*/

#ifndef HEATER_H
#define HEATER_H

#include "Config.h"

#define minmin(a,b) ((a)<(b)?(a):(b))
#define maxmax(a,b) ((a)>(b)?(a):(b))

class Heater
{
public:
  typedef struct __attribute__((packed)) {
    float temp_thresh       =   -40;
    float temp_gain         =   0.1666;    // degrees c of temp to percent duty cycle
    float comm_gain         =   1;    // hours of comm to percent duty cycle
    float cap_gain          =   1;    // voltage drop to percent duty cycle
    float cap_nominal       =   4.5;
    float max_duty          =   128;    // max duty cylce for heater in percent
  } Constants;

  typedef struct __attribute__((packed)) {
    float temp_int                 = 0;
    float voltage_supercap_average = 0;
    uint32_t rb_interval           = 0;
    uint32_t rb_last               = 0;
  } Input;

  float update(Input input);
  void updateConstants(Constants constants);
  Constants getConstants();

private:
  Constants constants;
};

#endif
