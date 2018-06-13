#include "Avionics.h"

/*
 * Function: processData
 * -------------------
 * This function updates the current data frame with derived values.
 */
bool Avionics::processData() {
  bool success = true;
  filter.enableSensors(data.BMP_1_ENABLE, data.BMP_2_ENABLE, data.BMP_3_ENABLE, data.BMP_4_ENABLE);
  filter.storeData(data.TIME, data.RAW_PRESSURE_1, data.RAW_PRESSURE_2, data.RAW_PRESSURE_3, data.RAW_PRESSURE_4,data.PRESS_BASELINE);
  data.TEMP_INT                   = filter.getTemp(data.RAW_TEMP_1, data.RAW_TEMP_2, data.RAW_TEMP_3, data.RAW_TEMP_4);
  data.PRESS                      = filter.getPressure();
  data.BMP_1_REJECTIONS           = filter.getNumRejections(1);
  data.BMP_2_REJECTIONS           = filter.getNumRejections(2);
  data.BMP_3_REJECTIONS           = filter.getNumRejections(3);
  data.BMP_4_REJECTIONS           = filter.getNumRejections(4);

  data.VOLTAGE_PRIMARY_MIN        = _min(data.VOLTAGE_PRIMARY, data.VOLTAGE_PRIMARY_MIN);

  data.VOLTAGE_SUPERCAP_AVG       = filter.getAvgVoltageSuperCap(data.VOLTAGE_SUPERCAP);
  data.VOLTAGE_SUPERCAP_MIN       = _min(data.VOLTAGE_SUPERCAP, data.VOLTAGE_SUPERCAP_MIN);
  data.CURRENT_TOTAL_AVG          = filter.getAvgCurrentSystem(data.CURRENT_TOTAL);
  data.CURRENT_TOTAL_MIN          = filter.getMinCurrentSystem();
  data.CURRENT_TOTAL_MAX          = filter.getMaxCurrentSystem();
  data.CURRENT_RB_AVG             = filter.getAvgCurrentRB(data.CURRENT_RB);
  data.CURRENT_RB_MAX             = filter.getMaxCurrentRB();
  data.CURRENT_MOTOR_VALVE_AVG    = filter.getAvgCurrentMotorValve(data.CURRENT_MOTOR_VALVE, (data.VALVE_STATE));
  data.CURRENT_MOTOR_VALVE_MAX    = filter.getMaxCurrentMotorValve();
  data.CURRENT_MOTOR_BALLAST_AVG  = filter.getAvgCurrentMotorBallast(data.CURRENT_MOTOR_BALLAST, (data.BALLAST_STATE));
  data.CURRENT_MOTOR_BALLAST_MAX  = filter.getMaxCurrentMotorBallast();
  data.CURRENT_PAYLOAD_AVG        = filter.getAvgCurrentPayload(data.CURRENT_PAYLOAD);
  data.CURRENT_PAYLOAD_MAX        = filter.getMaxCurrentPayload();

  data.LOOP_TIME_MAX              = _max(data.LOOP_TIME, data.LOOP_TIME_MAX);

  data.ALTITUDE_BAROMETER         = filter.getAltitude();
  data.ASCENT_RATE                = filter.getAscentRate();
  data.INCENTIVE_NOISE            = filter.getIncentiveNoise(data.BMP_1_ENABLE, data.BMP_2_ENABLE, data.BMP_3_ENABLE, data.BMP_4_ENABLE);
  float overpressure = analogRead(OP_PIN) * 1.2 / ((double)pow(2, 12)) * 3.2;
  float vref = analogRead(VR_PIN) * 1.2 / ((double)pow(2, 12)) * 3.2;
  float qty = (vref - 0.8)/2.;
  data.OVERPRESSURE = 498.1778/qty * (overpressure - (qty+0.5));

  data.OVERPRESSURE_VREF = vref;
  data.OVERPRESSURE_FILT          = op_filter.update(data.OVERPRESSURE);
  data.OVERPRESSURE_VREF_FILT          = op_vref_filter.update(data.OVERPRESSURE_VREF);
  if (data.ASCENT_RATE           >= 10) success = false;
  return success;
}

/*
 * Function: calcVitals
 * -------------------
 * This function calculates if the current state is within bounds.
 */
bool Avionics::calcVitals() {
  if(!data.SHOULD_REPORT) data.SHOULD_REPORT = (data.ASCENT_RATE >= 10);
  if(!data.MANUAL_MODE)   data.MANUAL_MODE   = (data.ASCENT_RATE >= 10);
  return true;
}

/*
 * Function: calcDebug
 * -------------------
 * This function calculates if the avionics is in debug mode.
 */
bool Avionics::calcDebug() {
  if(data.DEBUG_STATE && data.ALTITUDE_BAROMETER >= DEBUG_ALT) data.DEBUG_STATE = false;
  return true;
}

/*
 * Function: calcIncentives
 * -------------------
 * This function gets the updated incentives from the flight computer.
 */
bool Avionics::calcIncentives() {
  int numExecNow = numExecReset();
  bool success = true;
  computer.updateValveConstants(data.VALVE_SETPOINT, data.VALVE_VELOCITY_CONSTANT, data.VALVE_ALTITUDE_DIFF_CONSTANT, data.VALVE_LAST_ACTION_CONSTANT);
  computer.updateBallastConstants(data.BALLAST_SETPOINT, data.BALLAST_VELOCITY_CONSTANT, data.BALLAST_ALTITUDE_DIFF_CONSTANT, data.BALLAST_LAST_ACTION_CONSTANT);
  data.RE_ARM_CONSTANT   = computer.updateControllerConstants(data.BALLAST_ARM_ALT, data.INCENTIVE_THRESHOLD);
  data.VALVE_ALT_LAST    = computer.getAltitudeSinceLastVentCorrected(data.ALTITUDE_BAROMETER, data.VALVE_ALT_LAST);
  data.BALLAST_ALT_LAST  = computer.getAltitudeSinceLastDropCorrected(data.ALTITUDE_BAROMETER, data.BALLAST_ALT_LAST);
  data.VALVE_INCENTIVE   = computer.getValveIncentive(data.ASCENT_RATE, data.ALTITUDE_BAROMETER, data.VALVE_ALT_LAST);
  data.BALLAST_INCENTIVE = computer.getBallastIncentive(data.ASCENT_RATE, data.ALTITUDE_BAROMETER, data.BALLAST_ALT_LAST);
  if (!data.MANUAL_MODE && data.VALVE_INCENTIVE >= 1 && data.BALLAST_INCENTIVE >= 1) success = false;

  uint32_t INDEX = SPAG_CONTROLLER_INDEX;
  spagController.updateConstants(data.SPAG_CONSTANTS);
  SpaghettiController::Input spagInput;
  spagInput.h = data.ALTITUDE_BAROMETER;
  data.ACTIONS[INDEX] = 0;
  for (int k=0; k<numExecNow; k++) {
    spagController.update(spagInput);
    data.SPAG_STATE = spagController.getState();
    data.ACTIONS[INDEX] += spagController.getAction();
  }
  data.ACTION_TIME_TOTALS[2*INDEX] = data.ACTIONS[INDEX] < 0 ? data.ACTION_TIME_TOTALS[2*INDEX] - data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX];
  data.ACTION_TIME_TOTALS[2*INDEX+1] = data.ACTIONS[INDEX] > 0 ? data.ACTION_TIME_TOTALS[2*INDEX+1] + data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX+1];

  INDEX = SPAG2_CONTROLLER_INDEX;
  spag2Controller.updateConstants(data.SPAG2_CONSTANTS);
  SpaghettiController2::Input spag2Input;
  spag2Input.h = data.ALTITUDE_BAROMETER;
  data.ACTIONS[INDEX] = 0;
  for (int k=0; k<numExecNow; k++) {
    spag2Controller.update(spag2Input);
    data.SPAG2_STATE = spag2Controller.getState();
    data.ACTIONS[INDEX] += spag2Controller.getAction();
  }
  data.ACTION_TIME_TOTALS[2*INDEX] = data.ACTIONS[INDEX] < 0 ? data.ACTION_TIME_TOTALS[2*INDEX] - data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX];
  data.ACTION_TIME_TOTALS[2*INDEX+1] = data.ACTIONS[INDEX] > 0 ? data.ACTION_TIME_TOTALS[2*INDEX+1] + data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX+1];

  INDEX = LAS_CONTROLLER_INDEX;
  lasController.updateConstants(data.LAS_CONSTANTS);
  LasagnaController::Input lasInput;
  lasInput.h = data.ALTITUDE_BAROMETER;
  lasInput.op = isnan(data.OVERPRESSURE_FILT) ? 0 : data.OVERPRESSURE_FILT;
  data.ACTIONS[INDEX] = 0;
  for (int k=0; k<numExecNow; k++) {
    lasController.update(lasInput);
    data.LAS_STATE = lasController.getState();
    data.ACTIONS[INDEX] += lasController.getAction();
  }
  data.ACTION_TIME_TOTALS[2*INDEX] = data.ACTIONS[INDEX] < 0 ? data.ACTION_TIME_TOTALS[2*INDEX] - data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX];
  data.ACTION_TIME_TOTALS[2*INDEX+1] = data.ACTIONS[INDEX] > 0 ? data.ACTION_TIME_TOTALS[2*INDEX+1] + data.ACTIONS[INDEX] : data.ACTION_TIME_TOTALS[2*INDEX+1];
  //Serial.print("numExecNow: ");
  //Serial.println(numExecNow);
  return success;
}
