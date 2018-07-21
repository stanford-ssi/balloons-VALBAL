#include "PastaSim.h"



PastaSim::PastaSim() : 
	l_noise(0,0.003),
	v_noise(0,1.5)
{

	h = 13000;
	//gen.seed(std::time(0));
	gen.seed(1);

	conf.gtime.year = 2018;
    conf.gtime.month = 1;
    conf.gtime.day = 1;
    conf.gtime.hour = 10;
    conf.gtime.minute = 0;
    conf.gtime.second = 0;
}

double PastaSim::evolve(double action){
	time += 1/conf.freq;
	ctr++;

	if(ctr==conf.freq){
		sunpred.calcValues(conf.lon, conf.lat, conf.gtime, time);
		sunset_dldt = sunpred.estimated_dldt;
		ctr = 0;
	}

	action = action/1000;
	double dldt = action > 0 ? action*conf.b_dldt : action*conf.v_dldt;

	l += dldt + 1*l_noise(gen)/conf.freq + 1*sunset_dldt/conf.freq; 
	v = conf.klin*l + 1*v_noise(gen)/conf.freq;

	h += v/conf.freq;

	//l -= v/freq*5e-5;
	return h;
}