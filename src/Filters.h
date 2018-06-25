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

#include "Config.h"
#include <bitset>         // std::bitset
//#include <SD.h>

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
  void inputPressureDat(float time,Raw_Pressure P);
  void enableBmps(Bmp_Enable enable);
  float getAltitude(){return h_filtered};

private:
  float h_filtered;
  float 
};

#endif
