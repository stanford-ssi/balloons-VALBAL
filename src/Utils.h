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
#include "spa.h"

#define pi 3.14159

/*
Custom functions define here so they can be compiled on both x64 and ARM
*/
#define jankabs(x) ((x>0)-(x<0))*x

template<class T> const T pasta_abs(const T x)
{
    return ((x>0)-(x<0))*x;
}

template<class T> const T pasta_max(const T a, const T b)
{
    return (a < b) ? b : a;
}

template<class T> const T pasta_clamp(const T v, const T lo, const T hi)
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
  void setSS(float v);
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



class SunsetPredictor{
public:
  SunsetPredictor();
  void calcValues(float lon, float lat, float gps_tow, float gps_week);
  spa_data spa;
  float solar_elevation;
  float dsedt;
  float estimated_dldt;
  static constexpr float ang2 = 24;
  static constexpr float ang1 = -16;
  static constexpr int n_data = 100;
  static constexpr float sunset_data[n_data] = {   0.00000,   0.00082,   0.00138,   0.00174,   0.00195,   0.00206,   0.00217,   0.00235,   0.00259,   0.00290,   0.00327,   0.00369,   0.00416,   0.00468,   0.00526,   0.00590,   0.00661,   0.00738,   0.00821,   0.00902,   0.00976,   0.01036,   0.01082,   0.01129,   0.01188,   0.01261,   0.01347,   0.01442,   0.01534,   0.01606,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01638,   0.01606,   0.01534,   0.01442,   0.01347,   0.01261,   0.01188,   0.01129,   0.01082,   0.01036,   0.00976,   0.00902,   0.00821,   0.00738,   0.00661,   0.00590,   0.00526,   0.00468,   0.00416,   0.00369,   0.00327,   0.00290,   0.00259,   0.00235,   0.00217,   0.00206,   0.00195,   0.00174,   0.00138,   0.00082,   0.00000};
};


#endif
