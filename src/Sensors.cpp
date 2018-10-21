/*
  Stanford Student Space Initiative
  Balloons | VALBAL | December 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu
  Jesus Cervantes | cerjesus@stanford.edu

  File: Sensors.cpp
  --------------------------
  Implimentation of Sensors.h
*/

#include "Sensors.h"

/*const uint8_t sensor_pins[] = {ISENSE_RB, ISENSE_MOT, ISENSE_MAIN, ISENSE_PLD, ISENSE_SD, ISENSE_GPS};
const float sensor_gains[] = {1/0.54,1/0.54,1/0.54,1/0.54,1/5.4,1/5.4};
const int num_sensors = sizeof(sensor_pins);
volatile int sensor_accum[] = {0};
volatile int sensor_n = 0;*/

void read_sensors() {
	uint32_t t0 = micros();
	for (int i=0; i<num_sensors; i++) {
		for (int j=0; j<sensor_repeat; j++) {
			sensor_accum[i] += analogRead(sensor_pins[i]);
		}
	}
	sensor_accum[num_sensors] += (micros()-t0);
	sensor_n++;
}

/**********************************  SETUP  ***********************************/
/*
 * Function: init
 * -------------------
 * This function initializes the sensor hardware.
 */
bool Sensors::init() {
	analogReadAveraging(32);
  bool sucess = true;
  pinMode(SUPER_CAP_VOLTAGE,      INPUT);
	for (int i=0; i<num_sensors; i++) {
		pinMode(sensor_pins[i], INPUT);
	}
	for (unsigned int i=0; i<sizeof(sensor_accum); i++) {
		sensor_accum[i] = 0;
	}
	sensor_n = 0;
	sensorTimer.begin(read_sensors, 10000);
	sensorTimer.priority(128);

  //pinMode(EXT_TEMP_SENSOR,        INPUT);
  /*if (!bme1.begin()) {
    Serial.println("Could not initialize BMP280 sensor 1 in first test, check wiring!");
  }*/
  if (!bmp1.begin()) {
    Serial.println("Could not initialize BMP280 sensor 4x, check wiring!");
    sucess = false;
  }
  if (!bmp2.begin()) {
    Serial.println("Could not initialize BMP280 sensor 4y, check wiring!");
    sucess = false;
  }
  if (!bme2.begin()) {
    Serial.println("Could not initialize BMP280 sensor 2, check wiring!");
    sucess = false;
  }
  if (!bme2.begin()) {
    Serial.println("Could not initialize BMP280 sensor 2, check wiring!");
    sucess = false;
  }
  if (!bme1.begin()) {
    Serial.println("Could not initialize BMP280 sensor 1 again, check wiring!");
    sucess = false;
  }
  if (!bme3.begin()) {
    Serial.println("Could not initialize BMP280 sensor 3, check wiring!");
    sucess = false;
  }
  if (!bme4.begin()) {
    Serial.println("Could not initialize BMP280 sensor 4, check wiring!");
    sucess = false;
  }
  Serial.println(bme1.readPressure());
  Serial.println(bme2.readPressure());
  Serial.println(bme3.readPressure());
  Serial.println(bme4.readPressure());
  Serial.println(bmp1.readPressure());
  Serial.println(bmp2.readPressure());
	return true;
  /*Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.setDefaultTimeout(5000);

  // lol gdi
  int8_t ack = 0;
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CHANNEL_ENABLE_REG, LTC2991_ENABLE_ALL_CHANNELS);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_V1234_REG, 0x00);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_V5678_REG, 0x00);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_PWM_Tinternal_REG, LTC2991_REPEAT_MODE);*/
  return sucess;
}

/********************************  FUNCTIONS  *********************************/

/*
 * Function: getVoltageSuperCap
 * -------------------
 * This function gets the Super Cap voltage.
 */
float Sensors::getVoltageSuperCap() {
  voltageSuperCap = analogRead(SUPER_CAP_VOLTAGE) * 1.2 * 5.99 / (double)pow(2, 12);
  return voltageSuperCap;
}

float Sensors::getSensor(int sensor) {
	if (sensor_n != 0) return (((float)sensor_accum[sensor])/(sensor_n*sensor_repeat*(1<<12)))*sensor_gains[sensor]*1.2;
	else return 0;
}

float Sensors::getTime() {
	if (sensor_n != 0) return (((float)sensor_accum[num_sensors])/sensor_n);
	else return 0;
}

void Sensors::reset() {
	for (unsigned int i=0; i<sizeof(sensor_accum); i++) {
		sensor_accum[i] = 0;
	}
	sensor_n = 0;
}

/*
 * Function: getJoules
 * -------------------
 * This function gets the total joules.
 */
float Sensors::getJoules() {
  joules += (internalCurrentMonitor / 1000) * voltagePrimary * (millis() - lastJoulesCall) / 1000;
  lastJoulesCall = millis();
  return joules;
}

/*
 * Function: getDerivedTemp
 * -------------------
 * This function gets the extrapolated temperature.
 */
float Sensors::getDerivedTemp(uint8_t sensor) {
  double vA = analogRead(sensor) * 1.2 / (pow(2.0, 12.0));
  double x = log(vA * 100000.0 / (1.2 - vA));
  double a =   4.00141132e+02;
  double b =  -9.94189235e+01;
  double c =   1.16421122e+01;
  double d =  -9.42945754e-01;
  double e =   4.40544424e-02;
  double f =  -8.79058885e-04;
  return a + b * x + c * pow(x, 2) + d * pow(x, 3) + e * pow(x, 4) + f * pow(x, 5);
}

/*
 * Function: getRawTemp
 * -------------------
 * This function returns a raw reading from each of the sensors.
 */
float Sensors::getRawTemp(uint8_t sensor) {
  float value =  -1;
  if (sensor == 1) value = bme1.readTemperature();
  if (sensor == 2) value = bme2.readTemperature();
  if (sensor == 3) value = bme3.readTemperature();
  if (sensor == 4) value = bme4.readTemperature();
  return value;
}

/*
 * Function: getRawPressure
 * -------------------
 * This function returns a raw reading from each of the sensors.
 */
float Sensors::getRawPressure(uint8_t sensor) {
  float value =  -1;
  if (sensor == 1) value = bme1.readPressure();
  if (sensor == 2) value = bme2.readPressure();
  if (sensor == 3) value = bme3.readPressure();
  if (sensor == 4) value = bme4.readPressure();
  return value;
}
