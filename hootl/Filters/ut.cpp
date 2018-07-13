#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

#include "Filters.h"

using namespace std;

int main ()
{
	Filters filters;
	filters.update_voltage_primary(42);
	filters.update_voltage_primary(54);
	filters.update_voltage_primary(32);
	filters.update_current_rb(42);
	cout << filters.N_RB_VARS << endl;
	cout << filters.voltage_primary.min << endl;
	cout << filters.voltage_primary.max << endl;
	cout << filters.voltage_primary.avg << endl;
	filters.clear();
	cout << filters.voltage_primary.min << endl;
	cout << filters.voltage_primary.max << endl;
	cout << filters.voltage_primary.avg << endl;
}


