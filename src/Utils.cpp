/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  John Dean | deanjl@stanford.edu

  File: Utils.cpp
  --------------------------
  Implementation of Utils.h
*/

#include "Utils.h"
//#include <iostream>


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
void Biquad::setSS(float val){
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
void Biquad::setCoeffs(Coeffs coeffs){
  this->coeffs = coeffs;
  setSS(y[0]);
}

/*
 * Function: getSSGain
 * -------------------
 * Resturns the SS gain of the filter. If the filter is unstable,
 * you're fucked
 */
double Biquad::getSSGain(){
  double a_sum = 0;
  double b_sum = 0;
  for(int i = 0; i < 3; i++){
    a_sum += coeffs.a[i];
    b_sum += coeffs.b[i];
  }
  return b_sum/a_sum;
}

/*
 * Function: shiftBais
 * -------------------
 * Shifts thes steady state bias of the filter. Extremely useful for changing
 * the input commands to lead compensators
 */
void Biquad::shiftBias(float offset){
  float bias = offset*getSSGain();
  for(int i = 0; i < 3; i++){
    y[i] += bias;
    x[i] += offset;
  }
}



/*********************** DBIQUAD ************************/

/*
 * Function: update
 * -------------------
 * updates filter and returns a new output
 */
float DBiquad::update(float input){
  /* Roll back values */
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];
  y[2] = y[1];
  y[1] = y[0];

  /* calculate output */
  x[0] = input;
  y[0] = 1/coeffs.a[0] * (coeffs.b[0]*x[0] + coeffs.b[1]*x[1] + coeffs.b[2]*x[2] + coeffs.b[3]*x[3] - coeffs.a[1]*y[1] - coeffs.a[2]*y[2]);
  return y[0];
}


/*
 * Function: set_ss
 * -------------------
 * resets fitler to a steady state value
 */
void DBiquad::setSS(float val){
  for(int i = 0; i < 3; i++){
    x[i] = val;
    y[i] = val;
  }
  x[3] = val;
}

/*
 * Function: set_coeffs
 * -------------------
 * changes the coeficents of the biquad and sets to steady state.
 * Use with extreme caution
 */
void DBiquad::setCoeffs(Coeffs coeffs){
  this->coeffs = coeffs;
  setSS(y[0]);
}


/**********************************************/
/************* AdjustableLowpass **************/
/**********************************************/

AdjustableLowpass::AdjustableLowpass(float F0, float Q, float Fs) {
  this->F0 = F0;
  this->Q = Q;
  this->Fs = Fs;
  biquad.setCoeffs(calcCoeffs()); 
}

void AdjustableLowpass::setQ(float Q){
  this->Q = Q;
  biquad.setCoeffs(calcCoeffs()); 
}

void AdjustableLowpass::setCorner(float F0){
  this->F0 = F0;
  biquad.setCoeffs(calcCoeffs()); 
}

void AdjustableLowpass::setSampleRate(float Fs){
  this->Fs = Fs;
  biquad.setCoeffs(calcCoeffs());
}

float AdjustableLowpass::update(float input){
  return biquad.update(input);
}

Biquad::Coeffs AdjustableLowpass::calcCoeffs(){
  float w0 = 2 * pi * F0 / Fs;
  float alpha = sin(w0)/(2*Q);
  Biquad::Coeffs coeffs;
  coeffs.a[0] = 1+alpha;
  coeffs.a[1] = -2*cos(w0);
  coeffs.a[2] = 1-alpha;
  coeffs.b[0] = (1-cos(w0))/2;
  coeffs.b[1] = 1-cos(w0);
  coeffs.b[2] = (1-cos(w0))/2;
  return coeffs;
}



SunsetPredictor::SunsetPredictor(){
    // We don't use the date feilds so they are just some arbitrary day
    spa.year          = 2003;
    spa.month         = 10;
    spa.day           = 17;
    spa.hour          = 12;
    spa.minute        = 30;
    spa.second        = 30;
    spa.timezone      = -7.0;

    // Still not intirely sure what these should be TODO
    spa.delta_ut1     = 0;
    spa.delta_t       = 67;
    
    // Stanford Cooridinates default
    spa.longitude     = -122.1697;
    spa.latitude      = 37.4275;

    // Stuff that is way to detailed for what we need
    spa.elevation     = 0;
    spa.pressure      = 1000;
    spa.temperature   = 0;
    spa.slope         = 0;
    spa.azm_rotation  = 0;
    spa.atmos_refract = 0.5667;
    spa.function      = SPA_ZA;    
}
