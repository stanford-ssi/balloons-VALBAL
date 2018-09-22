#include "PastaSim.h"



PastaSim::PastaSim(int seed) : 
	l_noise(0,0.003/4.4721)
{
	gen.seed(seed);
	init();
}

PastaSim::PastaSim() : 
	l_noise(0,0.003/4.4721)
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
	if(time/(conf.sun_calc_interval*1000) >= sun_pred_ctr){
		sunpred.calcValues(conf.lon, conf.lat, conf.gtime, time/1000);
		sunset_dldt = sunpred.estimated_dldt;
		sun_pred_ctr++;
	}
	action = action/1000;
	float dldt = action > 0 ? action*conf.b_dldt : action*conf.v_dldt;

	l += dldt + 1*l_noise(gen)/4.4721/sqrt(conf.freq) + 1*sunset_dldt/conf.freq; 
	v = conf.klin*l;

	h += v/conf.freq;
	//l -= v/freq*5e-5;
	return h;
}