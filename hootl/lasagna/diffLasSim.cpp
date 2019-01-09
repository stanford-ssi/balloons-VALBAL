#include <iostream>
#include <fstream>
#include <iomanip>
#include "utils.h"
#include "LasagnaController.h"
#include "PastaSim.h"


int main(){
	const float freq = 1./60.;
	fstream o ("output.bin", std::fstream::out | std::fstream::binary);
	adept::Stack stack;
	PastaSim<adouble> sim;
	sim.conf.nightfall = false;
	sim.conf.freq = freq;
	sim.h = 13500;
	sim.l = 0;
	LasagnaController<adouble> las(freq);
	LasagnaController<adouble>::Constants c;
	c.tolerance = 1000;
	las.updateConstants(c);
	int dur = 60*60*24*1*freq;
	int act_sum = 0;
	adouble bal_sum = 0;
	printf("starting sim \n");
	for(int i = 0; i < dur; i++){
		LasagnaController<adouble>::Input input;
		adouble h = sim.evolve(double(las.getAction()));
		adouble bal = (las.getState()->deadband_effort > 0)*las.getState()->deadband_effort;
		bal_sum += bal;
		input.h_rel = h;
		input.h_abs = h;
		input.dldt_ext = 4*sim.sunset_dldt;
		las.update(input);
		if(i%int(freq*60*60) == 0){
			float t = float(i)/freq/60/60;
			printf("%f, %f, %f \n", t, VAL(sim.h), VAL(las.getState()->v1));
		}
	}
	printf("%f day flight, %f g ballast used \n",dur/60./60./24./freq,VAL(bal_sum));
	bal_sum.set_gradient(1.0);
	stack.compute_adjoint();
	printf("tol gradient: %f \n ", las.getConstants()->tolerance.get_gradient());
}