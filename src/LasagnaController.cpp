#include "LasagnaController.h"

LasagnaController::LasagnaController() :
  v1_filter({{1.0003141592601912L, -1.9999999013039569L, 0.99968584073980871L}, {4.93480216e-07L,   4.93480216e-07L,  -4.93480216e-07L,  -4.93480216e-07L}}),
  v2_filter({{1.0007479981811036L, -1.9999994404986428L, 0.9992520018188963L}, {2.79750679e-06L,  2.79750679e-06L, -2.79750679e-06L, -2.79750679e-06L}}),
  action_filter({{1L, -0.99994L, 0.0L}, {0.5L, 0.5L, -0.0L}})
{}

bool LasagnaController::update(Input input){ 

  if(isnan(input.dldt_ext)){
    input.dldt_ext = 0; 
  } else {
    float limit = 0.0001;
    input.dldt_ext = pasta_clamp(input.dldt_ext,-limit,limit);
  }

  switch(state.status){
    case PRELAUNCH:
      if(launch_h > constants.equil_h_thresh){        // this is the first time lasagna is called on startup
        if(input.h_rel > constants.equil_h_thresh){       // looks like we are already in flight ohp
          launch_h = 0;
          state.status = EQUIL;
        } else {
        launch_h = input.h_rel;
        }
      } else if(input.h_rel - launch_h > constants.launch_h_thresh) state.status = ASCENT;
      break;
    case ASCENT:
      if(input.h_rel > constants.equil_h_thresh) state.status = EQUIL;
      break;
    case EQUIL:
      break;
  }
  state.v_dldt = input.op*constants.v_dldt_a + constants.v_dldt_b; 
  state.v1 = v1_filter.update(input.h_rel);
  state.v2 = v2_filter.update(input.h_rel);
  state.v = (state.status==EQUIL) ? state.v1 : state.v2;
  float dldt_total = 0;
  if(state.status==EQUIL){
    float act_dldt = state.action > 0 ? float(state.action)*constants.kfuse*constants.b_dldt : float(state.action)*constants.kfuse*state.v_dldt*constants.kfuse_val;
    dldt_total = act_dldt + input.dldt_ext;
  }
  state.fused_v = state.v + action_filter.update(dldt_total);
  innerLoop(input.h_abs);
  if(state.status==PRELAUNCH) state.action = 0;
  return true;
}


void LasagnaController::innerLoop(float input_h){
  if(state.comp_ctr >= comp_freq*constants.freq){
    state.v_cmd = constants.k_h * (constants.h_cmd - input_h);
    state.v_cmd_clamped = pasta_clamp(state.v_cmd,-(constants.k_h*constants.ss_error_thresh+constants.v_limit),(constants.k_h*constants.ss_error_thresh+constants.v_limit));
    state.effort = constants.k_v * (state.v_cmd_clamped - state.fused_v);
    if((state.v_cmd_clamped - state.fused_v)*state.v_cmd_clamped < 0){
      state.effort = 0;
    }
    state.comp_ctr = 0;
  }

  float deadband_effort = 0;
  float thresh = constants.k_v*constants.k_h*constants.ss_error_thresh;
  //float thresh = 0;
  if(jankabs(state.effort)-thresh > 0){
    deadband_effort = state.effort + ((state.effort<0)-(state.effort>0))*thresh;
  }
  deadband_effort = pasta_clamp(deadband_effort,-state.v_dldt,constants.b_dldt);
  //if(state.v>constants.v_limit) deadband_effort = -max(-deadband_effort,float(0));
  //if(state.v<-constants.v_limit) deadband_effort = max(deadband_effort,float(0));
  state.effort_sum += deadband_effort/constants.freq;
  if(state.effort_sum >= constants.b_tmin*constants.b_dldt){
    state.action = constants.b_tmin;
    state.effort_sum -= constants.b_tmin*constants.b_dldt;
  } else if(-state.effort_sum >= constants.v_tmin*state.v_dldt){
    state.action = -constants.v_tmin;
    state.effort_sum -= -constants.v_tmin*state.v_dldt;
  } else {
    state.action = 0;
  }
  state.comp_ctr++;
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

LasagnaController::Constants LasagnaController::getConstants(){
  return constants;
}
