
#include "SpaghettiController.h"


SpaghettiController::SpaghettiController() :
  coeffs{{1, -0.30000000000000004, -0.4}, {2.8291628469154848e-05, -1.4117522606108269e-05, -1.4131668420342846e-05}},
  compensator(coeffs)
{
  state.effort = 0;
  state.v_T = HUGE_VALF;
  state.b_T = HUGE_VALF;
  state.v_ctr = 0;
  state.b_ctr = 0;
  state.action = 0;
}

bool SpaghettiController::update(Input input){

  /* get effort from compensator */
  if(state.comp_ctr >= comp_freq*constants.freq){
    state.effort = compensator.update(constants.h_cmd - input.h);
    state.comp_ctr = 0;
  }

  float rate = state.effort * constants.k;

  //TODO: Implement gain schedualing

  /* min/max thresholds*/
  rate = abs(rate) < constants.rate_max ? rate : ((0<rate) - (rate<0))*constants.rate_max;
  rate = abs(rate) > constants.rate_min ? rate : 0;

  /* calculate time intervals */
  state.v_T = rate < 0 ? abs(constants.v_dldt * constants.v_tmin / rate) : HUGE_VALF;
  state.b_T = rate > 0 ? abs(constants.b_dldt * constants.b_tmin / rate) : HUGE_VALF;

  /* reset counters if interval is inf*/
  state.v_ctr = state.v_T == HUGE_VALF ? 0 : state.v_ctr;
  state.b_ctr = state.b_T == HUGE_VALF ? 0 : state.b_ctr;

  /* check timers and act if necessary */
  if(float(state.v_ctr)/constants.freq >= state.v_T){
    state.action = -constants.v_tmin;
    state.v_ctr = 0;
  } else if(float(state.b_ctr)/constants.freq >= state.b_T){
    state.action = constants.b_tmin;
    state.b_ctr = 0;
  } else {
    state.action = 0;
  }

  /* increment counters */
  state.v_ctr++;
  state.b_ctr++;
  state.comp_ctr++;
  /* error checking is for the weak */
  return true;
}

void SpaghettiController::updateConstants(SpaghettiController::Constants constants){
  this->constants = constants;
}

int32_t SpaghettiController::getAction(){
  return state.action * 1000;
}

SpaghettiController::State SpaghettiController::getState(){
  return state;
}
