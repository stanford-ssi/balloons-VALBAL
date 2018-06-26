/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.h
  --------------------------
  Interface to guarenteed good derived values.
*/

#ifndef FILTERS_H
#define FILTERS_H

#include "Utils.h"

/* 
 * Yes, some of these values should be in a config file. However, I can't include that otherwise
 * we can't hootl, so live with it.
 */
const float mf = 1/60.0; 
const float H_FILTER_CORNER_DEFAULT = mf; 
const int N_V_FILTERS = 5; 
const float V_FILTERS_CORNER_DEFAULT[N_V_FILTERS] = {mf, mf/2, mf/5, mf/10, mf/15}; 
const float freq = 20;

typedef struct __attribute__((packed)) {
  float p1;
  float p2;
  float p3;
  float p4;
} Raw_Pressure;

typedef struct __attribute__((packed)) {
  float T1;
  float T2;
  float T3;
  float T4;
} Raw_Temp;

typedef struct __attribute__((packed)) {
  bool e1 = true;
  bool e2 = true;
  bool e3 = true;
  bool e4 = true;
} Bmp_Enable;

typedef struct __attribute__((packed)) {
  int c1 = 0;
  int c2 = 0;
  int c3 = 0;
  int c4 = 0;
} Bmp_Rejections;

class Filters {
public:
  Filters();
  bool init();
  void inputPressureDat(float time,Raw_Pressure P,Bmp_Enable enable);
  Bmp_Rejections getBmpRejections();
  float getTemp(Raw_Temp T);
  
  AdjustableLowpass h_filter;
  float h_filtered;
  float h_prefiltered_last;
  float v_raw;
  AdjustableLowpass v_filters[N_V_FILTERS];
  float v_filtered[N_V_FILTERS];
private:
};

#endif
