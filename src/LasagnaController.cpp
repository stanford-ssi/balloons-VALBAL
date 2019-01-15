#include "LasagnaController.h"
#include <cmath>

LasagnaController::LasagnaController() :
  v1_filter(1./60./16., 0.5, 20.),
  v2_filter(1./60./7.,  0.5, 20.),
  action_filter(1./60./16., 0.5, 20.)
{
}

LasagnaController::LasagnaController(float freq) : 
  v1_filter(1./60./16., 0.5, freq),
  v2_filter(1./60./7.,  0.5, freq),
  action_filter(1./60./16., 0.5, freq) 
{
  this->freq = freq;
}



bool LasagnaController::update(Input input){ 

  /**
   * Input protection for dtdl_ext
   */
  if(std::isnan(input.dldt_ext)){
    input.dldt_ext = 0; 
  } else {
    float limit = 0.6; // limit on what the max value for sunset dldt can be, in grams/s
    input.dldt_ext = pasta_clamp(input.dldt_ext,-limit,limit);
  }

  /**
   * Update both filtered velocities
   */
  if(!is_first_call){ 
    state.v1 = v1_filter.update((input.h_rel-state.h_rel_last)*freq);
    state.v2 = v2_filter.update((input.h_rel-state.h_rel_last)*freq);
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
      } else if(pasta_abs(input.h_rel - launch_h) > constants.launch_h_thresh) state.status = ASCENT;
      break;
    case ASCENT:
      if(input.h_rel > constants.equil_h_thresh) state.status = EQUIL;
      break;
    case EQUIL:
      break;
  }

  state.val_dldt = input.op*constants.val_dldt_slope + constants.val_dldt_intercept; 
  state.v = (state.status==EQUIL) ? state.v1 : state.v2; // v1 is low corner freqency, so it's used after equilibration

  /**
   * Predict changes in velocity from external dldt and dldt from actions. This is only done after the
   * balloon has equilibrated. We then sum all these delta v's (state.dv_sum). We add this sum to the 
   * fused_v; however we also subtract the dv_sum passed through the action_filter. This is because the
   * action filter has the same constants of the velocity filter, so it reflects the component of the predicted
   * dv_sum that should already be observed in the filtered velocity.
   */
  float dv_pred = 0;
  if(state.status==EQUIL){
    float act_dldt = state.action > 0 ? float(state.action)*constants.bal_dldt : float(state.action)*state.val_dldt*constants.kfuse_val;
    dv_pred = constants.k_drag*(act_dldt + input.dldt_ext/freq);
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

  bool override_gain = (constants.v_gain_override != 0) && (constants.h_gain_override != 0);
  float v_gain = override_gain ? constants.v_gain_override : 2.*constants.damping*sqrt(constants.gain / constants.k_drag);
  float h_gain = override_gain ? constants.h_gain_override : sqrt(constants.gain * constants.k_drag) / (2.*constants.damping);

  /**
   * compute effort pre-deadband
   */
  if(state.comp_ctr >= comp_freq*freq){
    state.v_cmd = h_gain * (constants.setpoint - input_h);
    state.v_cmd_clamped = pasta_clamp(state.v_cmd,-(h_gain*constants.tolerance+constants.v_limit),(h_gain*constants.tolerance+constants.v_limit));
    state.effort = v_gain * (state.v_cmd_clamped - state.fused_v);
    if((state.v_cmd_clamped - state.fused_v)*state.v_cmd_clamped < 0){
      state.effort = 0;
    }
    state.comp_ctr = 0;
  }

  /**
   * deadband
   */
  float deadband_effort = 0;
  float thresh = v_gain*h_gain*constants.tolerance;
  state.effort_ratio = state.effort / thresh;
  if(pasta_abs(state.effort)-thresh > 0){
    deadband_effort = state.effort + ((state.effort<0)-(state.effort>0))*thresh;
  }
  deadband_effort = pasta_clamp(deadband_effort,-state.val_dldt,constants.bal_dldt);

  /**
   * keeps track of a sum of the total desired effort (which is in kg/s since its a dldt)
   * from the controller. Once that effort is greater than the minimum vent or ballast ammout,
   * then an action happens. It could also be mutliples of the minimum ammouts, in which case multple
   * vent would happen. At 20Hz, there would never be multiple events in one control cycle, but at very low 
   * frequencies, it could happen.
   */
  state.effort_sum += deadband_effort/freq;
  if(state.effort_sum >= constants.bal_min_t*constants.bal_dldt){
    state.action = constants.bal_min_t*int(state.effort_sum/(constants.bal_min_t*constants.bal_dldt));
    state.effort_sum -= constants.bal_min_t*constants.bal_dldt*int(state.effort_sum/(constants.bal_min_t*constants.bal_dldt));
  } else if(-state.effort_sum >= constants.val_min_t*state.val_dldt){
    state.action = constants.val_min_t*int(state.effort_sum/(constants.val_min_t*state.val_dldt));
    state.effort_sum -= constants.val_min_t*state.val_dldt*int(state.effort_sum/(constants.val_min_t*state.val_dldt));
  } else {
    state.action = 0;
  }
  state.comp_ctr++;
}

void LasagnaController::updateConstants(Constants constants){
  //bool calc_gains = false;
  //if((this->constants.gain != constants.gain) || (this->constants.damping != constants.damping)){
  //  calc_gains = true;
  //}
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
