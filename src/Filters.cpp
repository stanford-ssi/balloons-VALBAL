/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Filters.cpp
  --------------------------
  Implementation of Filters.h
*/

#include "Filters.h"


Filters::Filters() : h_filter(H_FILTER_CORNER_DEFAULT, 0.5, 20) {
  for(int i = 0; i<N_V_FILTERS; i++){
    v_filters[i].setCorner(V_FILTERS_CORNER_DEFAULT[i]);
  }
  clear();
  for (uint16_t i = 0; i < VOLTAGE_BUFFER_SIZE; i++) superCapVoltageBuffer[i] = 5.0;
}

float Filters::calculate_altitude(float pressure) {
  float calculatedAltitude;
  if (pressure > 22632.1) calculatedAltitude = (44330.7 * (1 - pow(pressure / 101325.0, 0.190266)));
  else calculatedAltitude =  -6341.73 * log((0.176481 * pressure) / 22632.1);
  return calculatedAltitude;
}

void Filters::update_state(uint32_t time, float *pressures, DataFrame& data) {
  accepted_pressure = consensus_check(pressures, data.MAX_CONSENSUS_DEVIATION);
  accepted_velocity = velocity_check(time, pressures, data);

  for (int i=0; i<N_SENSORS; i++) {
    if (!bmps_enabled[i]) {
      data.BMP_REJECTIONS[i]++;
    }
  }

  incentive_noise = 0;
  bool force_accept = (accepted_velocity == 0) && ((time - last_sensor_time) > data.MAX_TIME_WITHOUT_SENSORS);
  if (!data.BMP_REJECTION_ENABLED || force_accept) { // fall back to whatever RB tells us
    last_pressure = 0;
    for (int i=0; i<N_SENSORS; i++) {
      bmps_enabled[i] = data.BMP_ENABLE[i];
      last_pressure += pressures[i];
    }
    last_pressure /= N_SENSORS;
    last_sensor_time = time;
  } else if (accepted_velocity != 0) {
    last_pressure = 0;
    int num = 0;
    for (int i=0; i<N_SENSORS; i++) {
      if (!bmps_enabled[i] && data.BMP_ENABLE[i]) incentive_noise += 1;
      bmps_enabled[i] = bmps_enabled[i] && data.BMP_ENABLE[i];
      if (bmps_enabled[i]) {
        last_pressure += pressures[i];
        num++;
      }
    }
    last_pressure /= num;
    last_sensor_time = time;
  }
  if (last_sensor_time == time) {
    float h_prefiltered = calculate_altitude(last_pressure);
    h_filtered = h_filter.update(h_prefiltered);
    data.ALTITUDE_PREFILT = h_prefiltered;
    data.ALTITUDE_BAROMETER = h_filtered;

    v_raw = (h_prefiltered - h_prefiltered_last)*freq;

    for(int i = 0; i<N_V_FILTERS; i++){
      v_filtered[i] = v_filters[i].update(v_raw);
    }
    data.ASCENT_RATE = v_filtered[1];
  }
}

int Filters::velocity_check(uint32_t time, float *pressures, const struct DataFrame &data) {
  if (first) {
    first = false;
    for (int i=0; i<N_SENSORS; i++) {
      fast_filters[i].setSS(pressures[i]);
      last_filtered[i] = pressures[i];
      last_accepted[i] = 0;
      vel_rejected[i] = false;
    }
  }
  int accepted = 0;
  for (int i=0; i<N_SENSORS; i++) {
    float slope = 940.9411 * pow(last_filtered[i], -0.8097);
    float ts = data.ERROR_REJECTION_DT + (time - last_accepted[i])/1000.;
    float vel = slope*(pasta_abs(pressures[i] - last_filtered[i])-data.ERROR_REJECTION_STD*5)/ts;
    if (vel <= data.ERROR_REJECTION_VEL && bmps_enabled[i]) {
      last_accepted[i] = time;
      if (vel_rejected[i]) {
        fast_filters[i].setSS(pressures[i]);
        last_filtered[i] = pressures[i];
        vel_rejected[i] = false;
      } else {
        last_filtered[i] = fast_filters[i].update(pressures[i]);
      }
      accepted++;
    } else {
      vel_rejected[i] = true;
      bmps_enabled[i] = false;
    }
  }
  return accepted;
}

int Filters::consensus_check(float *pressures, float max_dev) {
  int besti = 0;
  float mindist = 1e10;
  int maxsensors = 0;
  for (int i=0; i<N_SENSORS; i++) bmps_enabled[i] = false;
  const int num[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4}; // number of ones in corresponding binary numbers lol
  for (int current=1; current<(1 << N_SENSORS); current++) {
    if (num[current] < maxsensors || num[current] == 1) continue;
    float mxdist = 0;
    for (int i=0; i<N_SENSORS; i++) {
      if (!(current & (1 << i))) continue;
      for (int j=0; j<N_SENSORS; j++) {
        if (!(current & (1 << j))) continue;
        mxdist = pasta_max(mxdist, pasta_abs(pressures[i]-pressures[j]));
      }
    }
    if (mxdist > max_dev) continue;
    if (num[current] > maxsensors || (num[current] == maxsensors && mxdist < mindist)) {
      maxsensors = num[current];
      mindist = mxdist;
      besti = current;
    }
  }
  for (int i=0; i<N_SENSORS; i++) {
    bmps_enabled[i] = besti & (1 << i);
  }
  return maxsensors;
}

void Filters::update_current_motors(float current, bool valve, bool ballast) {
  if (valve) current_valve.update(current);
  if (ballast) current_ballast.update(ballast);
}

void Filters::clear() {
  for (int i=0; i<N_RB_VARS; i++) {
    rb_vars[i].clear();
  }
  current_ballast.clear();
  current_valve.clear();
}

float Filters::update_temperature(float *temps) {
  float temp = 0;
  int num = 0;
  for (int i=0; i<N_SENSORS; i++) {
    if (bmps_enabled[i]) {
      temp += temps[i];
      num++;
    }
  }
  return num == 0 ? 0 : temp/num; // num should never really be zero, but whatever
}

float Filters::update_voltage_supercap(float v) {
  superCapVoltageBuffer[superCapVoltageIndex] = v;
  superCapVoltageIndex++;
  superCapVoltageIndex %= VOLTAGE_BUFFER_SIZE;
  float superCapVoltageTotal = 0;
  for (uint16_t i = 0; i < VOLTAGE_BUFFER_SIZE; i++) superCapVoltageTotal += superCapVoltageBuffer[i];
  return superCapVoltageTotal / VOLTAGE_BUFFER_SIZE;
}
