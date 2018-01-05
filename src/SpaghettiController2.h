#ifndef SPAGHETTICONTROLLER2_H
#define SPAGHETTICONTROLLER2_H

//#include "Config.h"
#include "Utils.h"
#include <stdlib.h>

#define HUGE_VALF 2000000000.0

class SpaghettiController2
{
public:
  typedef struct {
    float effort;             // Command effort from compensator
    float v_T;                // Interval time between vent events
    float b_T;                // Interval time between ballast events
    unsigned int  v_ctr;                // valve interval counter
    unsigned int b_ctr;                // ballast interval counter
    unsigned int comp_ctr;
    int action;               // action command
    float ascent_rate;        // filtered ascent rate
    float fused_ascent_rate;
  } State;

  typedef struct {
    float h;                  // altidude
  } Input;

  typedef struct {
    float freq;               // control freqency
    float k;                  // gain modifier
    float b_dldt;             // balast dl/dt (kg/s)
    float v_dldt;             // valve dl/dt (kg/s))
    float b_tmin;               // minimum ballast event time
    float v_tmin;               // minimum valve event time
    float h_cmd;              // altidute comand
    float v_ss_error_thresh;
    float b_ss_error_thresh;
    float ascent_rate_thresh;
    float rate_max;
    float kfuse;
  } Constants;

  SpaghettiController2();
  bool update(Input input);
  void updateConstants(Constants constants);
  int getAction();
  State getState();
  Constants getConstants();
private:
  Constants constants;
  Biquad compensator;
  Biquad h_filter;
  DBiquad v_filter;
  Biquad action_filter;
  State state;
  double ss_gain;
  unsigned int comp_freq = 1;
  float spag1_tribute;  // if you allocate this memory, spaghetti will stop working
};

#endif
