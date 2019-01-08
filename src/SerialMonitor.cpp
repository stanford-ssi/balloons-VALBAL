// Same as Shitl.cpp first half
#include "Avionics.h"
#include "Arduino.h"
void Avionics::serialMonitorUpdate(){
  elapsedMicros te = 0;
  Serial.write(FSTART_SM);
  uint32_t t = data.TIME;
  char* b = (char*)(&t);
  Serial.write(b,4);
  int j = 0;
  // DO NOT REMOVE THE LINE BELOW, OR COPY IT ELSEWHERE IN THE FILE, ITS ACTUALLY IMPORTANT I SWEAR. -john bean
  // (the python script looks for that word to start parsing variable names we want to focus on)
  //diddlybop
  float report[30];
  report[j] = data.ALTITUDE_BAROMETER;  j++;
  report[j] = data.ASCENT_RATE;  j++;
  report[j] = data.ACTIONS[LAS_CONTROLLER_INDEX];  j++;
  report[j] = data.VALVE_QUEUE;  j++;
  report[j] = data.BALLAST_QUEUE;  j++;
  report[j] = data.LAS_STATE.action         ; j++;
  report[j] = data.LAS_STATE.v              ; j++;
  report[j] = data.LAS_STATE.v1             ; j++;
  report[j] = data.LAS_STATE.v2             ; j++;
  report[j] = data.LAS_STATE.fused_v        ; j++;
  report[j] = data.LAS_STATE.v_cmd          ; j++;
  report[j] = data.LAS_STATE.v_cmd_clamped  ; j++;
  report[j] = data.LAS_STATE.dv_sum         ; j++;
  report[j] = data.LAS_STATE.effort         ; j++;
  report[j] = data.LAS_STATE.effort_sum     ; j++;
  report[j] = data.LAS_STATE.effort_ratio   ; j++;
  report[j] = data.LAS_STATE.val_dldt         ; j++;
  report[j] = data.LAS_STATE.h_rel_last     ; j++;
  report[j] = data.LAS_CONSTANTS.gain              ; j++;
  report[j] = data.LAS_CONSTANTS.damping           ; j++;
  report[j] = data.LAS_CONSTANTS.v_gain_override   ; j++;
  report[j] = data.LAS_CONSTANTS.h_gain_override   ; j++;
  report[j] = data.LAS_CONSTANTS.bal_dldt          ; j++;
  report[j] = data.LAS_CONSTANTS.val_dldt_a        ; j++;
  report[j] = data.LAS_CONSTANTS.val_dldt_b        ; j++;
  report[j] = data.LAS_CONSTANTS.bal_tmin          ; j++;
  report[j] = data.LAS_CONSTANTS.val_tmin          ; j++;
  report[j] = data.LAS_CONSTANTS.setpoint          ; j++;
  report[j] = data.LAS_CONSTANTS.tolerance         ; j++;
  report[j] = data.LAS_CONSTANTS.k_drag            ; j++;
  report[j] = data.LAS_CONSTANTS.kfuse_val         ; j++;
  report[j] = data.LAS_CONSTANTS.v_limit           ; j++;
  report[j] = data.LAS_CONSTANTS.equil_h_thresh    ; j++;
  report[j] = data.LAS_CONSTANTS.launch_h_thresh   ; j++;
  b = (char*)(&report);
  Serial.write(b, sizeof(report));
}
