#include <iostream>
#include <fstream>
#include <iomanip>
#include "alias.h"
#include "SpaghettiController2.h"
#include "header.h"

#define FREQ 20

using namespace std;

SpaghettiController2::Constants getDefaultConstants();



int main ()
{
	SpaghettiController2::Constants constants = getDefaultConstants();
	SpaghettiController2 spag;
	spag.updateConstants(constants);
	int dur = 20*60*60*FREQ;
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	miniframe data;
	for(int i = 0; i < dur; i++){
		f.read((char*)&data, sizeof(miniframe));
		if (f.eofbit != 2) break;
		
		constants.h_cmd = data.SPAG_H_CMD;
		spag.updateConstants(constants);

		SpaghettiController2::Input input;
		input.h = data.ALTITUDE_BAROMETER;
		spag.update(input);

		SpaghettiController2::State state = spag.getState();
		float t = float(i)/FREQ/60;
		printf("%f, %f, %f \n", float(t), data.ALTITUDE_BAROMETER, state.b_T);
		float buf[6] = {data.ALTITUDE_BAROMETER, data.SPAG_H_CMD, state.effort, state.action, state.fused_ascent_rate, state.ascent_rate};
		o.write((char*)&buf, sizeof(float)*6);
	}
}

void f1(float* h_, int dur){
	float h = 10000;
	for(int i = 0;i < dur;i++){
		if(i < dur/4){
			
		} else if(i < dur/2){
			h += float(1)/FREQ;
		} else if(i < dur/(float(4))*3) {
			
		} else {
			h -= float(1)/FREQ;
		}
		h_[i] = h;
	}
}

SpaghettiController2::Constants getDefaultConstants(){
	SpaghettiController2::Constants constants;
	constants.freq = 20;
	constants.k = 1;
	constants.b_dldt = 0.0006;
	constants.v_dldt = 0.0030;
	constants.b_ss_error_thresh = 1000;
	constants.v_ss_error_thresh = 1000;
	constants.rate_max = 0.001;
	constants.b_tmin = 5;
	constants.v_tmin = 3;
	constants.h_cmd = 13000;
	constants.ascent_rate_thresh = 0.4;
	constants.kfuse = 7;
	constants.kfuse_v = 0.5;
	return constants;	
}

