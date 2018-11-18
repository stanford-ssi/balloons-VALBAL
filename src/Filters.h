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

#include "Utils.h"
#include "Data.h"

#ifdef JOHNSIM
  #include <stdint.h>
  #include <iostream>
  using namespace std;
#else
  #include <Arduino.h>
#endif

/*
 * Yes, some of these values should be in a config file. However, I can't include that otherwise
 * we can't hootl, so live with it.
 */
const float mf = 1/60.0;
const float H_FILTER_CORNER_DEFAULT = mf;
const float SUPERCAP_FILTER_CORNER = mf;
const int N_V_FILTERS = 5;
const int N_SENSORS = 4;
const float V_FILTERS_CORNER_DEFAULT[N_V_FILTERS] = {mf, mf/2, mf/5, mf/10, mf/15};
const float freq = 20;

/*
 * In loving memory of the
 *
 *            Excellent code standards put in place by Davy
 *                            2017 - 2018
 *
 * Brutally murdered by a Catalan gang in the streets of the city.
 * Gone, but not forgotten.
 *
 *      Keep Ithaka always in your mind.
 *      Arriving there is what you are destined for.
 *      But do not hurry the journey at all.
 *      Better if it lasts for years,
 *      so you are old by the time you reach the island,
 *      wealthy with all you have gained on the way,
 *      not expecting Ithaka to make you rich.
 */

#define _JOIN(X,Y) X##Y
#define JOIN(X,Y) _JOIN(X,Y)

#ifdef JOHNSIM
  #define DOOT(x) cout << x << endl;
#else
  #define DOOT(x) {}
#endif

#define RB_FILTERED(var) \
  void JOIN(update_, var)(float v) { \
    const int idx = __LINE__ - start - 1; \
    rb_vars[idx].update(v); \
  }; \
  var_t& var = rb_vars[__LINE__ - start - 1];

typedef struct {
  int num;
  float min;
  float max;
  float sum;
  float avg;

  void clear() {
    min = 1e10;
    max = -1e10;
    sum = 0;
    num = 0;
    avg = 0;
  }

  void update(float v) {
    if (v > max) max = v;
    if (v < min) min = v;
    sum += v;
    num++;
    avg = sum/num;
  }
} var_t;

class Filters {
public:
  Filters();
  bool init();
  void update_state(uint32_t time, float *pressures, struct DataFrame &data);
  float update_temperature(float *temperatures);
  float update_voltage_supercap(float);
  void update_current_motors(float, bool, bool);
  float get_incentive_noise(const struct DataFrame &data);
  void clear();
  void correctAltitude(struct DataFrame &data);
  static const int start = __LINE__;
  RB_FILTERED(voltage_primary)
  RB_FILTERED(current_total)
  RB_FILTERED(current_rb)
  RB_FILTERED(current_payload)
  RB_FILTERED(loop_time)
  static const int N_RB_VARS = __LINE__ - start - 1;
  var_t rb_vars[N_RB_VARS];

  AdjustableLowpass<> h_filter;
  float h_filtered;
  float h_prefiltered_last;
  float v_raw;
  AdjustableLowpass<> v_filters[N_V_FILTERS];
  float v_filtered[N_V_FILTERS];

  var_t current_ballast;
  var_t current_valve;

  float superCapVoltageBuffer[VOLTAGE_BUFFER_SIZE] = {0};
  uint16_t superCapVoltageIndex = 0;

  bool bmps_enabled[N_SENSORS];
  uint32_t bmps_last[N_SENSORS] = {0};

  AdjustableLowpass<> fast_filters[N_SENSORS];
  float last_filtered[N_SENSORS];
  uint32_t last_accepted[N_SENSORS];
  bool vel_rejected[N_SENSORS];
  uint32_t last_sensor_time = 0;
  float last_pressure = 0;

  int consensus_check(float *pressures, float maxdev);
  int velocity_check(uint32_t time, float *pressures, const struct DataFrame &data);

  float incentive_noise;

  int accepted_pressure, accepted_velocity;

  bool first = true;
private:
  float calculate_altitude(float pressure);
};

#endif
