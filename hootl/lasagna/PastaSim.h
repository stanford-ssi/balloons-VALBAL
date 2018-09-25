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
		unsigned int sun_calc_interval = 60; 
		float lon = -122.1697;
		float lat = 37.4275;
		SunsetPredictor::GPSTime gtime;
	} Config;
	Config conf;
	PastaSim(int seed);
	PastaSim();
	float evolve(float action);
	float h;
	float sunset_dldt = 0;
	unsigned int time = 0;
	SunsetPredictor sunpred;
	float v;
	float l;
	unsigned int sun_pred_ctr = 0;
private:
	void init();
	std::default_random_engine gen;
	std::normal_distribution<float> l_noise;
};


#endif