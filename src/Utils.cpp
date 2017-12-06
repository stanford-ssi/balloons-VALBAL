/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  John Dean | deanjl@stanford.edu

  File: Utils.cpp
  --------------------------
  Implementation of Utils.h
*/

#include "Utils.h"

/*********************** BIQUAD ************************/

/*
 * Function: update
 * -------------------
 * updates filter and returns a new output
 */
float Biquad::update(float input){
  /* Roll back values */
  x[2] = x[1];
  x[1] = x[0];
  y[2] = y[1];
  y[1] = y[0];

  /* calculate output */
  x[0] = input;
  y[0] = 1/coeffs.a[0] * (coeffs.b[0]*x[0] + coeffs.b[1]*x[1] + coeffs.b[2]*x[2] - coeffs.a[1]*y[1] - coeffs.a[2]*y[2]);
  return y[0];
}


/*
 * Function: set_ss
 * -------------------
 * resets fitler to a steady state value
 */
void Biquad::set_ss(float val){
  for(int i = 0; i < 3; i++){
    x[i] = val;
    y[i] = val;
  }
}

/*
 * Function: set_coeffs
 * -------------------
 * changes the coeficents of the biquad and sets to steady state.
 * Use with extreme caution
 */
void Biquad::set_coeffs(Coeffs coeffs){
  this->coeffs = coeffs;
  set_ss(y[0]);
}
