#include "PastaSim.h"



PastaSim::PastaSim(int seed) : 
	l_noise(0,0.0014)
{
	gen.seed(seed);
	init();
}

PastaSim::PastaSim() : 
	l_noise(0,0.0014)
{
	gen.seed(std::time(0));
	init();
}

void PastaSim::init(){
	h = 13000;
	conf.gtime.year = 2018;
    conf.gtime.month = 1;
    conf.gtime.day = 1;
    conf.gtime.hour = 10;
    conf.gtime.minute = 0;
    conf.gtime.second = 0;
}

float PastaSim::evolve(float  action){
	time += 1000/conf.freq;
	if((time/conf.sun_calc_interval >= sun_pred_ctr) && conf.nightfall){
		sunpred.calcValues(conf.lon, conf.lat, conf.gtime, time/1000);
		sunset_dldt = sunpred.estimated_dldt;
		sun_pred_ctr++;
	}
	action = action/1000;
	float dldt = action > 0 ? action*conf.bal_dldt : action*conf.val_dldt;

	l += dldt + 1*l_noise(gen)*1000./sqrt(conf.freq) + conf.nightfall*sunset_dldt/conf.freq; 
	v = conf.k_drag*l;

	h += v/conf.freq;
	//l -= v/freq*5e-5;
	return h;
}