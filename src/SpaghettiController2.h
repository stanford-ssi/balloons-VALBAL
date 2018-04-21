#ifndef SPAGHETTICONTROLLER2_H
#define SPAGHETTICONTROLLER2_H

//#include "Config.h"
#include "Utils.h"
#include <stdlib.h>

#define HUGE_VALF 2000000000.0

class SpaghettiController2
{
public:
  typedef struct __attribute__((packed)) {
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

  typedef struct __attribute__((packed)) {
    float h;                  // altidude
  } Input;

  typedef struct __attribute__((packed)) {
    float freq                          =               20;       // control freqency
    float k                             =              0.3;       // gain modifier
    float b_dldt                        =            0.0002;       // balast dl/dt (kg/s)
    float v_dldt                        =            0.001;       // valve dl/dt (kg/s)
    float rate_min                      =          0.00001;       // min dl/dt rate threshold (kg/s)
    float rate_max                      =            0.001;       // max dl/dt rate threshold (kg/s)
    float b_tmin                        =                4;       // minimum ballast event time
    float v_tmin                        =                4;       // minimum valve event time
    float h_cmd                         =            13500;       // altidute comand
    float v_ss_error_thresh             =              0.4;
    float b_ss_error_thresh             =             1000;
    float ascent_rate_thresh            =             1000;
    float kfuse                         =                7;
    float kfuse_v                       =              0.5;
  } Constants;

  SpaghettiController2();
  bool update(Input input);
  void updateConstants(SpaghettiController2::Constants constants);
  int getAction();
  SpaghettiController2::State getState();
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
