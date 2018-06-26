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

#include <math.h>

#define pi 3.14159

/*
Custom functions define here so they can be compiled on both x64 and ARM
*/
#define jankabs(x) ((x>0)-(x<0))*x

template<class T> const T& pasta_abs(const T& x)
{
    return ((x>0)-(x<0))*x;
}

template<class T> const T& pasta_max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

template<class T> const T& pasta_clamp( const T& v, const T& lo, const T& hi)
{
    return v < lo ? lo : hi < v ? hi : v;
}
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


/*
 * class: AdjustableLowpass
 * -------------------
 * Adjustable 2nd order IIR lowpass filter. Wrapper for biquad 
 */
class AdjustableLowpass{
public:
  AdjustableLowpass(float F0, float Q, float Fs);
  AdjustableLowpass();
  void setQ(float Q);
  void setCorner(float F0);
  void setSampleRate(float Fs);
  float update(float input);
private:
  Biquad::Coeffs calcCoeffs();
  Biquad biquad;
  float Q;
  float F0;
  float Fs;
};


#endif
