/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  John Dean | deanjl@stanford.edu

  File: Utils.h
  --------------------------
  Potentially usefull classes and functions
*/


#ifndef UTILS_H
#define UTILS_H

/*
 * class: Biquad
 * -------------------
 * Implementation of a biquad IIR filter.
 * Pray for stability.
 */
class Biquad {
public:
  typedef struct {
    float a[3];
    float b[3];
  } Coeffs;
  Biquad(): x{0,0,0},y{0,0,0},coeffs{{0,0,0},{0,0,0}}{}
  Biquad(Coeffs coeffs): x{0,0,0},y{0,0,0},coeffs(coeffs){}
  float update(float input);
  void set_ss(float val);
  void set_coeffs(Coeffs coeffs);
private:
  float x[3];
  float y[3];
  Coeffs coeffs;
};


#endif
