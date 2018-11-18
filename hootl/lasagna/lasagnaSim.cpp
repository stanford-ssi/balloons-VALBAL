#include <iostream>
#include <fstream>
#include <iomanip>
#include "LasagnaController.h"
#include "header.h"
#include "PastaSim.h"

#define CONTROLLER LasagnaController
#define LAS LasagnaController

using namespace std;


CONTROLLER::Constants getDefaultConstants();
CONTROLLER::Constants DefaultConstants();


void equil20Hz(){
	const float freq = 1;
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	PastaSim sim;
	sim.conf.nightfall = true;
	sim.conf.freq = freq;
	sim.h = 0;
	sim.l = 100;
	CONTROLLER las(freq);
	LasagnaController::Constants c;
	c.tolerance = 1000;
	las.updateConstants(c);
	printf("%f %f \n",las.getConstants().h_gain,las.getConstants().k_drag);
	miniframe data;
	float v_cmd = 0;
	int dur = 60*60*24*100*freq;
	int act_sum = 0;
	float bal_sum = 0;
	for(int i = 0; i < 60*60*20*4; i++){
		CONTROLLER::Input input;
		input.h_rel = sim.h;
		input.h_abs = sim.h;		
		las.update(input);
	}
	for(int i = 0; i < dur; i++){
		CONTROLLER::Input input;
		float h = sim.evolve(double(las.getAction()));
		float bal = (las.getAction() > 0)*las.getAction()/1000.*sim.conf.bal_dldt;
		bal_sum += bal;
		input.h_rel = h;
		input.h_abs = h;
		input.dldt_ext = 4*sim.sunset_dldt;
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
	printf("%f day flight, %f g ballast used \n",dur/60./60./24./freq,bal_sum);
}

void equilfreqtesting(){
	const float freq = 20;
	fstream f ("data.bin", std::fstream::in | std::fstream::binary);
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	PastaSim sim(1);
	sim.h = 5000;
	sim.l = -.1;
	sim.conf.freq = freq;
	CONTROLLER las(freq);
	printf("%f %f \n",1.0,las.getConstants().k_drag);
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

/**
 * This function demonstrates how to use the altitude controller simulator.
 */
void exampleSim(){ 

	const float freq = 20; //frequency of the simulation will be 20hz
	const float n_hours = 20; //number of hours to sim for
	const float n_steps = n_hours*60*60*freq; //number of steps to sim for
	
	fstream o("output.bin", std::fstream::out | std::fstream::binary); //Declare object to write files
	PastaSim sim(1); //Declare a simulation object and give it a seed of 1

	sim.h = 13500; //initialize altitude to 13.5km
	sim.l = 0; //initialize valbal with 0 free lift
	sim.conf.freq = freq;
	sim.conf.nightfall = false; //disable effects of nighfall

	/*******************************************************
	* 
	* Declare any values you need to store between loops here
	* 
	********************************************************/

	printf("Starting Simulation\n Time (hr), Altitude (km), Velocity (m/s)\n");
	for(int i = 0; i<n_steps; i++){

		float action = 0;
		float h = sim.h;

		/***************************************************************************************
		* 
		* Your code here. You may only use the varable "h" as the input to the control algorithm
		* You will set the variable action with the number of seconds to vent or ballast. 
		* (negative means to vent, positive means to ballast) 
		*
		***************************************************************************************/

		sim.evolve(action);

		/**
		 * log simulation varaibles ever second of simulation time
		 */
		if(i%(int(ceil(freq))) == 0){
			float buf[3] = {sim.h, sim.v, (float)sim.time};
			o.write((char*)&buf, sizeof(buf));
		}
		/**
		 * print simulation variables every 10 minutes of simulation
		 */
		if(i%int(ceil(freq*60*10)) == 0){
			printf("%10.2f,%14.1f,%15.1f \n", sim.time/1000./60./60., sim.h/1000., sim.v);
		}
	}
	printf("DONE\n");
}

int main ()
{
	exampleSim();
	//equil20Hz();
}
