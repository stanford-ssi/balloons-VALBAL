#ifndef lasagnaCONTROLLER2_H
#define lasagnaCONTROLLER2_H

//#include "Config.h"
#include "Utils.h"
#include <stdlib.h>
#ifdef JOHNSIM
#include <iostream>
#endif

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
    int action                 =   0;        // action command
    float v                    =   0;
    float v1                   =   0;
    float v2                   =   0;
    float fused_v              =   0;
    float v_cmd                =   0;
    float v_cmd_clamped        =   0;
    float dv_sum               =   0;
    float effort               =   0;
    float effort_sum           =   0;
    float effort_ratio         =   0;    
    float val_dldt               =   0;
    float h_rel_last           =   0;
    Status status              =   PRELAUNCH;
  } State;

  typedef struct __attribute__((packed)) {
    float h_abs = 0;
    float h_rel = 0;
    float op = 0;
    float dldt_ext = 0;
  } Input;

  typedef struct __attribute__((packed)) {
    float gain                 =   0.14/1000;   // Total gain magnititude (g / m)
    float damping              =   1.2;         // damping ratio (unitless)
    float v_gain               =   0;           // velocity gain (g/s / m/s) 
    float h_gain               =   0;           // altitude gain (m/s / km)
    float bal_dldt             =   0.666;       // balast dl/dt (g / s)
    float val_dldt_a           =   0;           
    float val_dldt_b           =   3;           // valve dl/dt (g / s))
    float bal_tmin             =   2;           // minimum ballast event time (s)
    float val_tmin             =   3;           // minimum valve event time (s)
    float setpoint             =   13500;       // altidute comand (m)
    float tolerance            =   750;         // comand tollerance (m)
    float k_drag               =   0.006;       // drag constant, (m/s / g)
    float kfuse_val            =   0.75;        // scale factor on effect of valve actions (unitless)
    float v_limit              =   0.5;         // velocity limit commanded by altitude loop (m/s)   
    float equil_h_thresh       =   10000;       // altitude where controller transitions to normal mode (m)
    float launch_h_thresh      =   300;         // change in altitide required to detect launch (m)
  } Constants;

  LasagnaController();
  LasagnaController(float freq);
  bool update(Input input);
  void updateConstants(Constants constants);
  int getAction();
  State getState();
  Constants getConstants();
private:
  float freq = 20;          // Control Freqency (Hz)
  void calcGains();
  void outerLoop();
  void innerLoop(float input_h);
  Constants constants;
  AdjustableLowpass v1_filter;
  AdjustableLowpass v2_filter;
  AdjustableLowpass action_filter;
  State state;
  unsigned int comp_freq = 1;
  float launch_h = 0;
  bool is_first_call = true;
};

#endif
