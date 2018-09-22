#include "LasagnaController.h"

LasagnaController::LasagnaController() :
  v1_filter(1./60./16., 0.5, 20.),
  v2_filter(1./60./7.,  0.5, 20.),
  action_filter(1./60./16., 0.5, 20.)
{}

LasagnaController::LasagnaController(float freq) : 
  v1_filter(1./60./16., 0.5, freq),
  v2_filter(1./60./7.,  0.5, freq),
  action_filter(1./60./16., 0.5, freq) 
{
  constants.freq = freq;
}

bool LasagnaController::update(Input input){ 

  /**
   * Input protection for dtdl_ext
   */
  if(isnan(input.dldt_ext)){
    input.dldt_ext = 0; 
  } else {
    float limit = 0.0001*4;
    input.dldt_ext = pasta_clamp(input.dldt_ext,-limit,limit);
  }

  /**
   * Update both filtered velocities
   */
  if(!is_first_call){ 
    state.v1 = v1_filter.update((input.h_rel-state.h_rel_last)*constants.freq);
    state.v2 = v2_filter.update((input.h_rel-state.h_rel_last)*constants.freq);
  }

  /**
   * Cases and transistions for various flight states
   */
  switch(state.status){
    case PRELAUNCH:
      if(is_first_call){        // this is the first time lasagna is called on startup
        if(input.h_rel > constants.equil_h_thresh){       // looks like we are already in flight ohp
          launch_h = 0;
          state.status = EQUIL;
        } else {
        launch_h = input.h_rel;
        }
        is_first_call = false;
      } else if(input.h_rel - launch_h > constants.launch_h_thresh) state.status = ASCENT;
      break;
    case ASCENT:
      if(input.h_rel > constants.equil_h_thresh) state.status = EQUIL;
      break;
    case EQUIL:
      break;
  }

  state.v_dldt = input.op*constants.v_dldt_a + constants.v_dldt_b; 
  state.v = (state.status==EQUIL) ? state.v1 : state.v2; // v1 is low corner freqency, so it's used after equilibration

  /**
   * Predict changes in velocity from external dldt and dldt from actions. This is only done after the
   * balloon has equilibrated. We then sum all these delta v's (state.dv_sum). We add this sum to the 
   * fused_v, however we also substract the dv_sum passed through the action_filter. This is becuase the 
   * action filter has the same constants of the velocity filter, so it reflects the component of the predicted
   * dv_sum that should already be observed in the filtered velocity.
   */
  float dv_pred = 0;
  if(state.status==EQUIL){
    float act_dldt = state.action > 0 ? float(state.action)*constants.b_dldt : float(state.action)*state.v_dldt*constants.kfuse_val;
    dv_pred = constants.kfuse*(act_dldt + input.dldt_ext/constants.freq);
  }
  state.dv_sum += dv_pred;  
  state.fused_v = state.v + state.dv_sum - action_filter.update(state.dv_sum);

  /**
   * Now the inner loop is called which computes the actions given the estimated velocity and altitude
   */
  innerLoop(input.h_abs);

  if(state.status==PRELAUNCH) state.action = 0;  // if we haven't launched yet, don't do any actions

  state.h_rel_last = input.h_rel;
  return true;
}


void LasagnaController::innerLoop(float input_h){

  /**
   * compute effort pre-deadband
   */
  if(state.comp_ctr >= comp_freq*constants.freq){
    state.v_cmd = constants.k_h * (constants.h_cmd - input_h);
    state.v_cmd_clamped = pasta_clamp(state.v_cmd,-(constants.k_h*constants.ss_error_thresh+constants.v_limit),(constants.k_h*constants.ss_error_thresh+constants.v_limit));
    state.effort = constants.k_v * (state.v_cmd_clamped - state.fused_v);
    if((state.v_cmd_clamped - state.fused_v)*state.v_cmd_clamped < 0){
      state.effort = 0;
    }
    state.comp_ctr = 0;
  }

  /**
   * deadband
   */
  float deadband_effort = 0;
  float thresh = constants.k_v*constants.k_h*constants.ss_error_thresh;
  if(jankabs(state.effort)-thresh > 0){
    deadband_effort = state.effort + ((state.effort<0)-(state.effort>0))*thresh;
  }
  deadband_effort = pasta_clamp(deadband_effort,-state.v_dldt,constants.b_dldt);

  /**
   * keeps track of a sum of the total desired effort (which is in kg/s since its a dldt)
   * from the controller. Once that effort is greater than the minimum vent or ballast ammout,
   * then an action happens. It could also be mutliples of the minimum ammouts, in which case multple
   * vent would happen. At 20Hz, there would never be multiple events in one control cycle, but at very low 
   * frequencies, it could happen.
   */
  state.effort_sum += deadband_effort/constants.freq;
  if(state.effort_sum >= constants.b_tmin*constants.b_dldt){
    state.action = constants.b_tmin*int(state.effort_sum/(constants.b_tmin*constants.b_dldt));
    state.effort_sum -= constants.b_tmin*constants.b_dldt*int(state.effort_sum/(constants.b_tmin*constants.b_dldt));
  } else if(-state.effort_sum >= constants.v_tmin*state.v_dldt){
    state.action = constants.v_tmin*int(state.effort_sum/(constants.v_tmin*state.v_dldt));
    state.effort_sum -= constants.v_tmin*state.v_dldt*int(state.effort_sum/(constants.v_tmin*state.v_dldt));
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
