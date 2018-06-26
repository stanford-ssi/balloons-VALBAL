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

Filters::Filters() : h_filter(H_FILTER_CORNER_DEFAULT,0.5,20) {
  for(int i = 0; i<N_V_FILTERS; i++){
    v_filters[i].setCorner(V_FILTERS_CORNER_DEFAULT[i]);
  }
}

void Filters::inputPressureDat(float time,Raw_Pressure P,Bmp_Enable enable){

  float h_prefiltered;  // Joan make this the output of the consensus + velocity checks
  
  h_filtered = h_filter.update(h_prefiltered);
  v_raw = (h_prefiltered - h_prefiltered_last)*freq;

  for(int i = 0; i<N_V_FILTERS; i++){
    v_filtered[i] = v_filters[i].update(v_raw);
  }
}