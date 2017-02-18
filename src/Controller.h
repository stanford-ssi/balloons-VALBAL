/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Ella Hofmann-Coyle | ellahofm@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Controller.h
  --------------------------
  Interface to feedback control algorythm.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Config.h"
#include <math.h>

class Controller {
public:
/**********************************  SETUP  ***********************************/
  bool init();
/********************************  FUNCTIONS  *********************************/
  float getValveIncentive(float valveKpConstant, float valveKdConstant, float valveKiConstant, double ascentRate);
  float getBalastIncentive(float balastKpConstant, float balastKdConstant, float balastKiConstant, double ascentRate);
private:
/*********************************  OBJECTS  **********************************/
};

#endif
