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
void runSim(CONTROLLER::Constants constants,int hours);

fstream o ("outputMC.bin", std::fstream::out | std::fstream::binary);

struct {
	int hours;
	int size;
	CONTROLLER::Constants constants;
} hd;

int main ()
{
	hd.constants = getDefaultConstants();
	hd.hours = 1000;
	hd.size = sizeof(CONTROLLER::Constants);
	float gain[20] = {0.03981072,  0.04493985,  0.0507298 ,  0.05726572,  0.06464372,
        0.07297228,  0.08237387,  0.09298675,  0.10496696,  0.11849069,
        0.13375678,  0.15098972,  0.17044291,  0.19240242,  0.21719114,
        0.24517359,  0.27676124,  0.31241857,  0.35266992,  0.39810717};
	for(int i = 0; i < 20; i++){
		printf("%f, %d\n",gain[i],i);
		hd.constants.k_v = gain[i]*1e-3;
		hd.constants.k_h = gain[i]*1.5e-3;
		o.write((char*)&hd,sizeof(hd));
		runSim(hd.constants,hd.hours);
	}
}


void runSim(CONTROLLER::Constants constants,int hours){
	PastaSim sim;
	CONTROLLER las;
	las.updateConstants(constants);
	
	miniframe data;
	float v_cmd = 0;
	int dur = hours*60*60*FREQ;
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
			float h = sim.h;
			o.write((char*)&h, sizeof(float));
			o.write((char*)&act_sum,sizeof(act_sum));
		}
		/*
		if(i%(FREQ*60*60) == 0){
			float t = float(i)/FREQ/60/60;
			printf("%f, %f, %f \n", t, sim.h, state.effort);
		}
		*/
	}	
}


#if 1
CONTROLLER::Constants getDefaultConstants(){
	float gain = .1;
	CONTROLLER::Constants constants;
    constants.freq 			= 20;               // call frequency
    constants.k_v 			= gain*1e-3;                  // gain modifier
    constants.k_h			= gain*1.5e-3;
    constants.b_dldt 		= 0.0006;             // balast dl/dt (kg/s)
    constants.v_dldt 		= 0.0006;             // valve dl/dt (kg/s))
    constants.b_tmin 		= 5;               // minimum ballast event time
    constants.v_tmin 		= 5;               // minimum valve event time
    constants.h_cmd 		= 14000;              
    constants.kfuse 		= 7;
    constants.kfuse_val 	= 1;
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
