#ifndef PASTASIM_H
#define PASTASIM_H
#include <random>
#include <ctime>
class PastaSim
{
public:
	PastaSim();
	double evolve(double action);
	double klin;
	double v_dldt;
	double b_dldt;
	double freq;
	double h;
private:
	double v;
	double l;
	std::default_random_engine gen;
	std::normal_distribution<double> l_noise;
	std::normal_distribution<double> v_noise;

};


#endif