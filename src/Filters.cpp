/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.cpp
  --------------------------
  Implementation of Filters.h
*/

#include "Filters.h"

Filters::Filters() : h_filter(H_FILTER_CORNER_DEFAULT, 0.5, 20),
                     supercap_filter(SUPERCAP_FILTER_CORNER, 0.5, 20){
  for(int i = 0; i<N_V_FILTERS; i++){
    v_filters[i].setCorner(V_FILTERS_CORNER_DEFAULT[i]);
  }
  clear();
}

void Filters::update_state(uint32_t time, float *presssures, const DataFrame& data){
/*
  float h_prefiltered;  // Joan make this the output of the consensus + velocity checks

  h_filtered = h_filter.update(h_prefiltered);
  v_raw = (h_prefiltered - h_prefiltered_last)*freq;

  for(int i = 0; i<N_V_FILTERS; i++){
    v_filtered[i] = v_filters[i].update(v_raw);
  }*/
}

void Filters::update_current_motors(float current, bool valve, bool ballast) {
  if (valve) current_valve.update(current);
  if (ballast) current_ballast.update(ballast);
}

void Filters::clear() {
  for (int i=0; i<N_RB_VARS; i++) {
    rb_vars[i].clear();
  }
  current_ballast.clear();
  current_valve.clear();
  voltage_supercap.clear();
}

float Filters::update_temperature(float *temps) {
  return 0;
}

float Filters::get_incentive_noise(const DataFrame& data) {
  return 0;
}

float Filters::update_voltage_supercap(float v) {
  float out = supercap_filter.update(v);
  voltage_supercap.update(out);
  return out;
}
