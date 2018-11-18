#include "Avionics.h"
#include "Arduino.h"

#define NUM_FLOATS_SHITLED 13

float shitlavg = 0;
int shitln = 0;

void Avionics::shitlUpdate(){
  elapsedMicros te = 0;
  Serial.write(FSTART);
  Serial.flush();
  uint32_t t = data.TIME;
  char* b = (char*)(&t);
  Serial.write(b,4);
  Serial.flush();
  int j = 0;
  // DO NOT REMOVE THE LINE BELOW, OR COPY IT ELSEWHERE IN THE CODE, ITS ACTUALLY IMPORTANT I SWEAR. -john bean
  //diddlybop
  float report[52];
  report[j] = data.ALTITUDE_BAROMETER;  j++;
  report[j] = data.ASCENT_RATE;  j++;
  report[j] = data.ACTIONS[LAS_CONTROLLER_INDEX];  j++;
  report[j] = data.VALVE_QUEUE;  j++;
  report[j] = data.BALLAST_QUEUE;  j++;
  report[j] = data.LAT_GPS     ;  j++;
  report[j] = data.LONG_GPS    ;  j++;
  report[j] = data.ALTITUDE_GPS;  j++;
  report[j] = data.HEADING_GPS ;  j++;
  report[j] = data.SPEED_GPS   ;  j++;
  report[j] = data.NUM_SATS_GPS   ;  j++;
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
  report[j] = data.LAS_CONSTANTS.v_gain            ; j++;
  report[j] = data.LAS_CONSTANTS.h_gain            ; j++;
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
  report[j] = data.ESTIMATED_DLDT; j++;
  report[j] = data.SOLAR_ELEVATION; j++;
  report[j] = data.DSEDT; j++;
  report[j] = filter.v_filtered[0]; j++;
  report[j] = filter.v_filtered[1]; j++;
  report[j] = filter.v_filtered[2]; j++;
  report[j] = filter.v_filtered[3]; j++;
  report[j] = filter.v_filtered[4]; j++;
  report[j] = data.LAT_GPS_MANUAL; j++;
  report[j] = data.LONG_GPS_MANUAL; j++;
  report[j] = data.GPS_MANUAL_MODE; j++;
  report[j] = data.GPS_MANUAL_MODE_OVERRIDE; j++;
  report[j] = data.GPS_LAST_NEW; j++;
  report[j] = sunsetPredictor.spa.jc; j++;
  report[j] = sunsetPredictor.spa.latitude; j++;
  report[j] = sunsetPredictor.spa.longitude; j++;
  b = (char*)(&report);
  Serial.write(b, sizeof(report));
  Serial.flush();
  const int len = sizeof(float)*NUM_FLOATS_SHITLED;
  char bytes[len];
  char flags;
  float vals[NUM_FLOATS_SHITLED];
  /* 
   * 1 byte of flags, followed by NUM_FLOATS_SHITLED
   * followed by 1 byte for num_sats_gps.
  */
  while(true){
    if(Serial.available() == len+2){
      flags = Serial.read();
      for(int i = 0;i < len; i++){
        bytes[i]= Serial.read();
        //Serial.printf("%x,",bytes[i]);
      }
      memcpy(vals,bytes,len);
      data.NUM_SATS_GPS = (uint8_t)Serial.read();
      break;
    }
  }
  if(flags & (1<<0)){
    Serial.println("FLAG");
    int index;
    float value;
    while(true){
      if(Serial.available()==8){
        for(int i=0;i<4;i++){
          bytes[i]=Serial.read();
        }
        memcpy(&index,bytes,4); 
        for(int i=0;i<4;i++){
          bytes[i]=Serial.read();
        }
        memcpy(&value,bytes,4);
        break;
      }
    }
    updateConstant(index,value);
  }
  data.RAW_TEMP_1 = (isnan(vals[4]) ? data.RAW_TEMP_1 : vals[4]);
  data.RAW_TEMP_2 = (isnan(vals[5]) ? data.RAW_TEMP_2 : vals[5]);
  data.RAW_TEMP_3 = (isnan(vals[6]) ? data.RAW_TEMP_3 : vals[6]);
  data.RAW_TEMP_4 = (isnan(vals[7]) ? data.RAW_TEMP_4 : vals[7]);
  data.RAW_PRESSURE_1 = (isnan(vals[0]) ? data.RAW_PRESSURE_1: vals[0]);
  data.RAW_PRESSURE_2 = (isnan(vals[1]) ? data.RAW_PRESSURE_2: vals[1]);
  data.RAW_PRESSURE_3 = (isnan(vals[2]) ? data.RAW_PRESSURE_3: vals[2]);
  data.RAW_PRESSURE_4 = (isnan(vals[3]) ? data.RAW_PRESSURE_4: vals[3]);
  data.LAT_GPS          = vals[8];  
  data.LONG_GPS         = vals[9];
  data.ALTITUDE_GPS     = vals[10];
  data.HEADING_GPS      = vals[11];
  data.SPEED_GPS        = vals[12];
  Serial.print("SHITL TIME: ");
  Serial.println(te);
  shitlavg += te;
  shitln++;
  Serial.print("SHITL AVG: ");
  Serial.println(shitlavg/shitln);
}
