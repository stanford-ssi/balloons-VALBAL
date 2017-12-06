/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  Claire Huang | chuang20@stanford.edu

  File: ControllerPrototype.h
  --------------------------
  Abstract controller specifying what functions all controllers must implement.
*/

#ifndef CONTROLLERPROTOTYPE_H
#define CONTROLLERPROTOTYPE_H

#include "Config.h"

class ControllerPrototype {
public:
  /**********************************  SETUP  ***********************************/
    virtual bool  init() = 0;

  /********************************  FUNCTIONS  *********************************/
  virtual void updateConstants() = 0;
  virtual void update() = 0;
  virtual void getAction() = 0;
  virtual void getState() = 0;
};

#endif
