#ifndef SPAGHETTICONTROLLER_H
#define SPAGHETTICONTROLLER_H

#include "Config.h"
#include "Utils.h"
#include <stdlib.h>

class SpaghettiController
{
public:
  typedef struct {
    float effort;             // Command effort from compensator
    float v_T;                // Interval time between vent events
    float b_T;                // Interval time between ballast events
    uint32_t v_ctr;                // valve interval counter
    uint32_t b_ctr;                // ballast interval counter
    uint32_t comp_ctr;
    int32_t action;               // action command
  } State;

  typedef struct {
    float h;                  // altidude
  } Input;

  typedef struct {
    float freq;               // control freqency
    float k;                  // gain modifier
    float b_dldt;             // balast dl/dt (kg/s)
    float v_dldt;             // valve dl/dt (kg/s)
    float rate_min;           // min dl/dt rate threshold (kg/s)
    float rate_max;           // max dl/dt rate threshold (kg/s)
    float b_tmin;               // minimum ballast event time
    float v_tmin;               // minimum valve event time
    float h_cmd;              // altidute comand
  } Constants;

  SpaghettiController();
  bool update(Input input);
  void updateConstants(SpaghettiController::Constants constants);
  int32_t getAction();
  SpaghettiController::State getState();
private:
  Biquad::Coeffs coeffs;
  Constants constants;
  Biquad compensator;
  SpaghettiController::State state;
  uint32_t comp_freq = 1;
};

#endif
