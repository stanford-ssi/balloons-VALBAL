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

#define OLD_DATA

using namespace std;


CONTROLLER::Constants getDefaultConstants();
CONTROLLER::Constants DefaultConstants();


int main ()
{
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	
	PastaSim sim;

	CONTROLLER las;
	CONTROLLER::Constants con;
	las.updateConstants(con);
	printf("%f %f \n",las.getConstants().freq,las.getConstants().kfuse);
	miniframe data;
	float v_cmd = 0;
	int dur = 10*60*60*FREQ;
	int act_sum = 0;

	for(int i = 0; i < dur; i++){
		f.read((char*)&data, sizeof(miniframe));
		if (f.eofbit != 2) break;
		CONTROLLER::Input input;
		input.h = data.ALTITUDE_BAROMETER;
		las.update(input);
		CONTROLLER::State state = las.getState();
		act_sum += state.action;
		if(i%(FREQ) == 0){
			float buf[6] = {data.ALTITUDE_BAROMETER, v_cmd,state.effort, state.effort_sum, state.fused_v, state.v};
			o.write((char*)&buf, sizeof(float)*6);
			o.write((char*)&act_sum,sizeof(act_sum));
		}
		if(i%(FREQ*60*60) == 0){
			float t = float(i)/FREQ/60/60;
			printf("%f, %f, %f \n", t, data.ALTITUDE_BAROMETER, state.effort);
		}
	}
}

