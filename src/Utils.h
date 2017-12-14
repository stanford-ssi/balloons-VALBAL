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
    double a[3];
    double b[3];
  } Coeffs;
  Biquad(): x{0.0,0.0,0.0},y{0.0,0.0,0.0} ,coeffs{{0.0,0.0,0.0},{0.0,0.0,0.0}}{}
  Biquad(Coeffs coeffs): x{0.0,0.0,0.0},y{0.0,0.0,0.0},coeffs(coeffs){}
  float update(float input);
  void setSS(float val);
  void setCoeffs(Coeffs coeffs);
  double getSSGain();
  void shiftBias(float offset);
private:
  double x[3];
  double y[3];
  Coeffs coeffs;
};

class DBiquad {
public:
  typedef struct {
    double a[3];
    double b[4];
  } Coeffs;
  DBiquad(): x{0},y{0} ,coeffs{{0},{0}} {}
  DBiquad(Coeffs coeffs): x{0},y{0},coeffs(coeffs){}
  float update(float input);
  void setSS(float val);
  void setCoeffs(Coeffs coeffs);
private:
  double x[4];
  double y[3];
  Coeffs coeffs;
};

#endif
