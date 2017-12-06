#ifndef SPAGHETTICONTROLLER_H
#define SPAGHETTICONTROLLER_H

#include "Utils.h"
#include <stdlib.h>

#define HUGE_VALF 2000000000.0

class SpaghettiController
{
public:
  typedef struct {
    float effort;     // Command effort from compensator
    float v_T;        // Interval time between vent events
    float b_T;        // Interval time between ballast events
    int v_ctr;        // valve interval counter
    int b_ctr;        // ballast interval counter
    int action;       // action command
  } State;

  typedef struct {
    float h;          // altidude
    float h_cmd;      // altidute comand
  } Input;

  typedef struct {
    float freq;               // control freqency
    Biquad::Coeffs coeffs;    // Biquad coeffs
    float k;                  // gain modifier
    float b_dldt;             // balast dl/dt (kg/s)
    float v_dldt;             // valve dl/dt (kg/s)
    float rate_min;           // min dl/dt rate threshold (kg/s)
    float rate_max;           // max dl/dt rate threshold (kg/s)
    int b_tmin;               // minimum ballast event time
    int v_tmin;               // minimum valve event time
  } Constants;

  SpaghettiController(Constants constants);
  bool update(Input input);
  void updateConstants(Constants constants);
  int getAction();
  State getState();
  Constants getConstants();
private:
  Constants constants;
  Biquad compensator;
  State state;
};

#endif
