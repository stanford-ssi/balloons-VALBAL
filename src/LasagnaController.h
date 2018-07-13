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
  enum Status {
    PRELAUNCH,
    ASCENT,
    EQUIL,
  };

  typedef struct __attribute__((packed)) {
    unsigned int comp_ctr      =   0;
    int action                 =   0;               // action command
    float v                    =   0;
    float v1                   =   0;        
    float v2                   =   0;
    float fused_v              =   0;
    float effort               =   0;
    float effort_sum           =   0;
    float v_cmd                =   0;
    float v_cmd_clamped        =   0;
    float v_dldt               =   0;
    Status status              =   PRELAUNCH;
  } State;

  typedef struct __attribute__((packed)) {
    float h  = 0;                  // altidude
    float op = 0;
  } Input;

  typedef struct __attribute__((packed)) {
    float freq                 =   20;        // control freqency
    float k_v                  =  .5*1e-3;      // velocity gain
    float k_h                  =  .5*1.5e-3;    // altitude gain
    float b_dldt               =   0.0002;    // balast dl/dt (kg/s)
    float v_dldt_a             =   0;
    float v_dldt_b             =   0.0030;    // valve dl/dt (kg/s))
    float b_tmin               =   2;         // minimum ballast event time
    float v_tmin               =   3;         // minimum valve event time
    float h_cmd                =   13500;     // altidute comand
    float kfuse                =   7;
    float kfuse_val            =   0.5;
    float ss_error_thresh      =   750;
    float v_limit              =   0.5;
    float equil_h_thresh       =   10000;      //altitude where controller transitions to normal mode
    float launch_h_thresh      =   300;
  } Constants;

  LasagnaController();
  bool update(Input input);
  void updateConstants(Constants constants);
  int getAction();
  State getState();
  Constants getConstants();
private:
  void outerLoop();
  void innerLoop(float input_h);
  Constants constants;
  DBiquad v1_filter;
  DBiquad v2_filter;
  Biquad action_filter;
  State state;
  unsigned int comp_freq = 1;
  float launch_h = 20000;     //starting altitude (yes this looks skect ik)
};

#endif
