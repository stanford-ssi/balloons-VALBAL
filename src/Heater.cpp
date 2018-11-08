/*
  Stanford Student Space Initiative
  Balloons | VALBAL | May 2018

  Jonathan Zwiebel | jzwiebel@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu

  File: Heater.cpp
  --------------------------
  Implementation of Heater.h
*/

#include "Heater.h"

float Heater::update(Input input) {
  float duty = 0;
  duty += maxmax((constants.temp_thresh - input.temp_int) * constants.temp_gain,0);
  duty += maxmax((constants.cap_nominal - input.voltage_supercap_average) * constants.cap_gain, 0);
  duty += maxmax((((float)(millis() - input.rb_last)) - 2 * input.rb_interval) / 1000. / 3600. * constants.comm_gain, 0);

  duty = minmin(1, duty);
  duty = maxmax(0, duty);

  int dutyint = duty * constants.max_duty;

  //analogWrite(36, dutyint);
  return dutyint;
}

void Heater::updateConstants(Constants constants) {
  this->constants = constants;
}
