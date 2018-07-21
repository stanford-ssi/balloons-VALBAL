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
		double klin = 6;
		double v_dldt = 0.003;
		double b_dldt = 0.0008;
		double freq = 20;
		double lon = -122.1697;
		double lat = 37.4275;
		SunsetPredictor::GPSTime gtime;
	} Config;
	Config conf;
	PastaSim();
	double evolve(double action);
	double h;
	double sunset_dldt = 0;
	double time = 0;

	SunsetPredictor sunpred;
private:
	double v;
	double l;
	int ctr = 0;
	std::default_random_engine gen;
	std::normal_distribution<double> l_noise;
	std::normal_distribution<double> v_noise;

};


#endif