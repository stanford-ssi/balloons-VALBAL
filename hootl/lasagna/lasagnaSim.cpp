#include <iostream>
#include <fstream>
#include <iomanip>
#include "LasagnaController.h"
#include "SpaghettiController2.h"
#include "header.h"
#include "PastaSim.h"

#define FREQ 1
#define CONTROLLER LasagnaController
#define LAS LasagnaController
#define SPAG SpaghettiController2

using namespace std;


CONTROLLER::Constants getDefaultConstants();
CONTROLLER::Constants DefaultConstants();


void equil20Hz(){
	const int freq = 20;
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	PastaSim sim(1);
	sim.h = 0;
	sim.l = 0.1;
	CONTROLLER las;
	printf("%f %f \n",las.getConstants().freq,las.getConstants().kfuse);
	miniframe data;
	float v_cmd = 0;
	int dur = 60*60*24*4*freq;
	int act_sum = 0;
	for(int i = 0; i < 60*60*20*4; i++){
		CONTROLLER::Input input;
		input.h_rel = sim.h;
		input.h_abs = sim.h;		
		las.update(input);
	}
	for(int i = 0; i < dur; i++){
		CONTROLLER::Input input;
		float h = sim.evolve(double(las.getAction()));
		input.h_rel = h;
		input.h_abs = h;
		input.dldt_ext = sim.sunset_dldt;
		las.update(input);
		CONTROLLER::State state = las.getState();
		act_sum += state.action;
		if(i%(freq) == 0){
			float buf[6] = {sim.h, v_cmd,state.effort, state.effort_sum, state.fused_v, state.v};
			o.write((char*)&buf, sizeof(float)*6);
			o.write((char*)&act_sum,sizeof(act_sum));
		}
		if(i%(freq*60*60) == 0){
			float t = float(i)/freq/60/60;
			printf("%f, %f, %f \n", t, sim.h, state.v1);
		}
	}
}

void equilfreqtesting(){
	const float freq = 1/60.;
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	PastaSim sim(1);
	sim.h = 12000;
	sim.l = 0.1;
	sim.conf.freq = freq;
	CONTROLLER las(freq);
	printf("%f %f \n",las.getConstants().freq,las.getConstants().kfuse);
	miniframe data;
	float v_cmd = 0;
	int dur = 60*60*24*4*freq;
	int act_sum = 0;
	for(int i = 0; i < 60*60*20*4; i++){
		CONTROLLER::Input input;
		input.h_rel = sim.h;
		input.h_abs = sim.h;		
		las.update(input);
	}
	for(int i = 0; i < dur; i++){
		CONTROLLER::Input input;
		float h = sim.evolve(double(las.getAction()));
		input.h_rel = h;
		input.h_abs = h;
		input.dldt_ext = sim.sunset_dldt*3;
		las.update(input);
		CONTROLLER::State state = las.getState();
		act_sum += state.action;
		if(i%(int(ceil(freq))) == 0){
			float buf[6] = {sim.h, v_cmd,state.effort, state.effort_sum, state.fused_v, state.v};
			o.write((char*)&buf, sizeof(float)*6);
			o.write((char*)&act_sum,sizeof(act_sum));
		}
		if(i%int(freq*60*60) == 0){
			float t = float(i)/freq/60/60;
			printf("%f, %f, %f \n", t, sim.h, state.v1);
		}
	}
}


int main ()
{
	equilfreqtesting();
}