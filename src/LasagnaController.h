#ifndef lasagnaCONTROLLER2_H
#define lasagnaCONTROLLER2_H

//#include "Config.h"
#include "Utils.h"
#include <stdlib.h>

#ifdef JOHNSIM
#include <iostream>
#endif

#define HUGE_VALF 2000000000.0

class LasagnaController
{
public:
  typedef struct {
    unsigned int comp_ctr      =   0;
    int action                 =   0;               // action command
    float ascent_rate          =   0;        // filtered ascent rate
    float fused_ascent_rate    =   0;
    float effort               =   0;
    float effort_sum           =   0;
    float v_cmd                =   0;
  } State;

  typedef struct {
    float h;                  // altidude
  } Input;

  typedef struct {
    float freq                 =   20;        // control freqency
    float k_v                  =   1e-3;      // velocity gain
    float k_h                  =   1.5e-3;    // altitude gain
    float b_dldt               =   0.00019;    // balast dl/dt (kg/s)
    float v_dldt               =   0.0030;    // valve dl/dt (kg/s))
    float b_tmin               =   10;         // minimum ballast event time
    float v_tmin               =   5;         // minimum valve event time
    float h_cmd                =   13000;     // altidute comand
    float kfuse                =   7;
    float kfuse_val            =   0.5;
    float ss_error_thresh      =   750;
  } Constants;

  LasagnaController();
  bool update(Input input);
  void updateConstants(Constants constants);
  int getAction();
  State getState();
  Constants getConstants();
private:
  Constants constants;
  DBiquad v_filter;
  Biquad action_filter;
  State state;
  unsigned int comp_freq = 1;
};

#endif
