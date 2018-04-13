#include "LasagnaController.h"

LasagnaController::LasagnaController() :
  constants{0},
  v_filter({{1.0003141592601912, -1.9999999013039569, 0.99968584073980871}, {4.93480216e-07,   4.93480216e-07,  -4.93480216e-07,  -4.93480216e-07}}),
  action_filter({{1, -0.99994, 0.0}, {0.5, 0.5, -0.0}})
{
  state.effort = 0;
  state.effort_sum = 0;
  state.action = 0;
  state.ascent_rate = 0;
  state.fused_ascent_rate = 0;
  state.v_cmd = 0;
}

bool LasagnaController::update(Input input){

  /* biquad for veloctiy with fused action effect estimation*/
  state.ascent_rate = v_filter.update(input.h);
  float action_effect = float(state.action)*constants.kfuse;
  state.fused_ascent_rate = state.ascent_rate + action_filter.update(state.action > 0 ? action_effect*constants.b_dldt : action_effect*constants.v_dldt*constants.kfuse_val);

  if(state.comp_ctr >= comp_freq*constants.freq){
    state.v_cmd = constants.k_h * (constants.h_cmd - input.h);
    state.effort = constants.k_v * (state.v_cmd - state.fused_ascent_rate);
    state.comp_ctr = 0;
  }


  float deadband_effort = 0;
  float thresh = constants.k_v*constants.k_h*constants.ss_error_thresh;
  //float thresh = 0;
  if(jankabs(state.effort)-thresh > 0){
    deadband_effort = state.effort + ((state.effort<0)-(state.effort>0))*thresh;
  }
  state.effort_sum += deadband_effort*constants.freq;

  if(state.effort_sum >= constants.b_tmin*constants.b_dldt*constants.kfuse){
    state.action = constants.b_tmin;
    state.effort_sum -= constants.b_tmin*constants.b_dldt*constants.kfuse;
  } else if(-state.effort_sum >= constants.v_tmin*constants.v_dldt*constants.kfuse){
    state.action = -constants.v_tmin;
    state.effort_sum -= -constants.v_tmin*constants.v_dldt*constants.kfuse;
  } else {
    state.action = 0;
  }
  state.comp_ctr++;
  return true;
}

void LasagnaController::updateConstants(Constants constants){
  if(this->constants.h_cmd != constants.h_cmd){

  }
  this->constants = constants;
}

int LasagnaController::getAction(){
  return state.action * 1000;
}

LasagnaController::State LasagnaController::getState(){
  return state;
}
