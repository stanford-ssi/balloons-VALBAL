#include "Avionics.h"
#include "Arduino.h"
void Avionics::shitlUpdate(){
  elapsedMicros te = 0;
  Serial.write(FSTART);
  uint32_t t = data.TIME;
  char* b = (char*)(&t);
  Serial.write(b,4);
  int j = 0;
  // DO NOT REMOVE THE LINE BELOW, OR COPY IT ELSEWHERE IN THE CODE, ITS ACTUALLY IMPORTANT I SWEAR. -john bean
  //diddlybop
  float report[30];
  report[j] = data.ALTITUDE_BAROMETER;  j++;
  report[j] = data.ASCENT_RATE;  j++;
  report[j] = data.ACTIONS[LAS_CONTROLLER_INDEX];  j++;
  report[j] = data.VALVE_QUEUE;  j++;
  report[j] = data.BALLAST_QUEUE;  j++;
  report[j] = data.LAS_STATE.comp_ctr;  j++;
  report[j] = data.LAS_STATE.action;  j++;
  report[j] = data.LAS_STATE.v;  j++;
  report[j] = data.LAS_STATE.v1;  j++;
  report[j] = data.LAS_STATE.v2;  j++;
  report[j] = data.LAS_STATE.fused_v;  j++;
  report[j] = data.LAS_STATE.effort;  j++;
  report[j] = data.LAS_STATE.effort_sum;  j++;
  report[j] = data.LAS_STATE.v_cmd;  j++;
  report[j] = data.LAS_STATE.status;  j++;
  report[j] = data.LAS_CONSTANTS.freq;  j++;
  report[j] = data.LAS_CONSTANTS.k_v;  j++;
  report[j] = data.LAS_CONSTANTS.k_h;  j++;
  report[j] = data.LAS_CONSTANTS.b_dldt;  j++;
  report[j] = data.LAS_CONSTANTS.v_dldt_a;  j++;
  report[j] = data.LAS_CONSTANTS.v_dldt_b;  j++;
  report[j] = data.LAS_CONSTANTS.b_tmin;  j++;
  report[j] = data.LAS_CONSTANTS.v_tmin;  j++;
  report[j] = data.LAS_CONSTANTS.h_cmd;  j++;
  report[j] = data.LAS_CONSTANTS.kfuse;  j++;
  report[j] = data.LAS_CONSTANTS.kfuse_val;  j++;
  report[j] = data.LAS_CONSTANTS.ss_error_thresh;  j++;
  report[j] = data.LAS_CONSTANTS.v_limit;  j++;
  report[j] = data.LAS_CONSTANTS.equil_h_thresh;  j++;
  report[j] = data.LAS_CONSTANTS.launch_h_thresh;  j++;
  b = (char*)(&report);
  Serial.write(b, sizeof(report));
  const int len = sizeof(float)*8;
  char bytes[len];
  char flags;
  float vals[8];
  while(true){
    if(Serial.available() == len+1){
      flags = Serial.read();
      for(int i = 0;i < len; i++){
        bytes[i]= Serial.read();
        //Serial.printf("%x,",bytes[i]);
      }
      memcpy(vals,bytes,len);
      break;
    }
  }
  if(flags & (1<<0)){
    Serial.println("OOO FLAG");
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
  data.RAW_TEMP_1 = (isnan(vals[0]) ? data.RAW_TEMP_1 : vals[0]);
  data.RAW_TEMP_2 = (isnan(vals[1]) ? data.RAW_TEMP_2 : vals[1]);
  data.RAW_TEMP_3 = (isnan(vals[2]) ? data.RAW_TEMP_3 : vals[2]);
  data.RAW_TEMP_4 = (isnan(vals[3]) ? data.RAW_TEMP_4 : vals[3]);
  data.RAW_PRESSURE_1 = (isnan(vals[4]) ? data.RAW_PRESSURE_1: vals[4]);
  data.RAW_PRESSURE_2 = (isnan(vals[5]) ? data.RAW_PRESSURE_2: vals[5]);
  data.RAW_PRESSURE_3 = (isnan(vals[6]) ? data.RAW_PRESSURE_3: vals[6]);
  data.RAW_PRESSURE_4 = (isnan(vals[7]) ? data.RAW_PRESSURE_4: vals[7]);
  Serial.print("SHITL TIME: ");
  Serial.println(te);
}