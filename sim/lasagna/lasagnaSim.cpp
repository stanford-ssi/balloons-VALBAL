#include <iostream>
#include <fstream>
#include <iomanip>
#include "alias.h"
#include "LasagnaController.h"
#include "SpaghettiController2.h"
#include "header.h"
#include "PastaSim.h"


#define FREQ 20
#define CONTROLLER LasagnaController
#define LAS LasagnaController
#define SPAG SpaghettiController2

using namespace std;


CONTROLLER::Constants getDefaultConstants();
CONTROLLER::Constants DefaultConstants();


int main ()
{
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	
	PastaSim sim;

	CONTROLLER::Constants constants = getDefaultConstants();
	CONTROLLER las;
	las.updateConstants(constants);
	
	miniframe data;
	float v_cmd = 0;
	int dur = 100*60*60*FREQ;
	int act_sum = 0;
	for(int i = 0; i < 60*60*20*4; i++){
		CONTROLLER::Input input;
		input.h = sim.h;		
		las.update(input);
	}
	for(int i = 0; i < dur; i++){
		CONTROLLER::Input input;
		input.h = sim.evolve(double(las.getAction()));
		las.update(input);
		CONTROLLER::State state = las.getState();
		act_sum += state.action;
		if(i%(FREQ) == 0){
			float buf[6] = {sim.h, v_cmd,state.effort, state.effort_sum, state.fused_ascent_rate, state.ascent_rate};
			o.write((char*)&buf, sizeof(float)*6);
			o.write((char*)&act_sum,sizeof(act_sum));
		}
		if(i%(FREQ*60*60) == 0){
			float t = float(i)/FREQ/60/60;
			printf("%f, %f, %f \n", t, sim.h, state.effort);
		}
	}
}

#if 1
CONTROLLER::Constants getDefaultConstants(){
	float gain = .3;
	CONTROLLER::Constants constants;
    constants.freq 			= 20;               // call frequency
    constants.k_v 			= gain*1e-3;                  // gain modifier
    constants.k_h			= gain*1.5e-3;
    constants.b_dldt 		= 0.0006;             // balast dl/dt (kg/s)
    constants.v_dldt 		= 0.006;             // valve dl/dt (kg/s))
    constants.b_tmin 		= 5;               // minimum ballast event time
    constants.v_tmin 		= 5;               // minimum valve event time
    constants.h_cmd 		= 14000;              
    constants.kfuse 		= 7;
    constants.kfuse_val 	= .5;
    constants.ss_error_thresh = 750;
	return constants;
}
#else
CONTROLLER::Constants getDefaultConstants(){
	CONTROLLER::Constants constants;
	constants.freq = 20;
	constants.k = 1;
	constants.b_dldt = 0.0006;
	constants.v_dldt = 0.0030;
	constants.b_ss_error_thresh = 600;
	constants.v_ss_error_thresh = 600;
	constants.rate_max = 0.001;
	constants.b_tmin = 5;
	constants.v_tmin = 5;
	constants.h_cmd = 14000;
	constants.ascent_rate_thresh = 0.4;
	constants.kfuse = 7;
	constants.kfuse_v = 0.5;
	return constants;	
}

#endif
