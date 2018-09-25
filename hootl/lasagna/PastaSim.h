#ifndef PASTASIM_H
#define PASTASIM_H
#include <random>
#include <ctime>
#include "Utils.h"
class PastaSim
{
public:
	typedef struct {
		bool nightfall = 1;
		float klin = 6;
		float v_dldt = 0.003;
		float b_dldt = 0.0008;
		float freq = 20;
		unsigned int sun_calc_interval = 60000; // interval for how often sun posisition is calculated, in ms
		float lon = -122.1697;
		float lat = 37.4275;
		SunsetPredictor::GPSTime gtime;
	} Config;
	Config conf;
	PastaSim(int seed);
	PastaSim();
	float evolve(float action);
	float h;							// sim altitude in m
	float sunset_dldt = 0;
	unsigned int time = 0;				// sim time in ms
	SunsetPredictor sunpred;
	float v;							// sim velocity in m/s
	float l;							// sim balloon lift in kg 
	unsigned int sun_pred_ctr = 0;
private:
	void init();
	std::default_random_engine gen;
	std::normal_distribution<float> l_noise;
};


#endif