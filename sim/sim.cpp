#include <iostream>
#include "alias.h"
#include "SpaghettiController2.h"


float profile1(float t){
	return 
}

int main ()
{
	SpaghettiController2 spaghetti;
	SpaghettiController2::Constants constants;
	constants.freq = 20;
	constants.k = 0.35;
	constants.b_dldt = 0.0006;
	constants.v_dldt = 0.03;
	constants.b_ss_error_thresh = 500;
	constants.v_ss_error_thresh = 500;
	constants.b_tmin = 5;
	constants.v_tmin = 3;
	constants.h_cmd = 13000;
	constants.ascent_rate_thresh = 0.4;
	spaghetti.updateConstants(constants);
	float h = 0;
	for(int t = 0; t < 10*20*60*60; t++){
		float h = profile1(float(t)/20)
		SpaghettiController2::Input input;
		input.h = h;
		spaghetti.update(input);
		SpaghettiController2::State state = spaghetti.getState();
		if(t%20 == 0) printf("%f, %f, %f, %f \n", state.b_T, state.v_T, state.effort, state.ascent_rate);
		//if(spaghetti.getAction()!=0)printf("action: %f\n",spaghetti.getAction());
	}
}

