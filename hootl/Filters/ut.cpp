#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

#include "Filters.h"

using namespace std;

char *load(const char *name) {
	char path[100] = "arrs/";
	strcat(path, name);
	strcat(path, ".bin");
    FILE *dat = fopen(path, "rb");
    if (dat == NULL) {
        cout << "unable to open file" << endl;
        exit(1);
    }
    fseek(dat, 0L, SEEK_END);
    size_t sz = ftell(dat);
    rewind(dat);
    char *data = (char*)malloc(sz);
    if (!fread(data, sz, 1, dat)) {
        cout << "unable to read file" << endl;
        fclose(dat);
        free(data);
        exit(1);
    }
    fclose(dat);
	return data;
}

void write_to_file(const char *fn, void *ptr, size_t bytes) {
    FILE *f = fopen(fn, "wb");
    if (!fwrite(ptr, bytes, 1, f)) {
        printf("Unable to log to file.\n");
    }
    fclose(f);
}

int main ()
{
	const int N = 4750916;
	uint32_t *times = (uint32_t*)load("t");
	float *p1 = (float*)load("p1");
	float *p2 = (float*)load("p2");
	float *p3 = (float*)load("p3");
	float *p4 = (float*)load("p4");
	cout << p1[0] << " " << p1[300] << endl;
	float *out = (float*)malloc(N*4);
	Filters filters;
	struct DataFrame data;
	for (int i=0; i<N; i++) {
		float pressures[] = {p1[i], p2[i], p3[i], p4[i]};
		uint32_t time = times[i];
		filters.update_state(time, pressures, data);
		out[i] = data.ALTITUDE_PREFILT;
	}
	write_to_file("out.bin", out, N*4);
	/*filters.update_voltage_primary(42);
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
	float pressures[] = {800, 200, 300, 500};
	//float pressures[] = {800, 1600, 2400, 3200};
	float temperatures[] = {10,20,30,40};
	struct DataFrame data;
	filters.update_state(0, pressures, data);
	cout << filters.bmps_enabled[0] << endl;
	cout << filters.bmps_enabled[1] << endl;
	cout << filters.bmps_enabled[2] << endl;
	cout << filters.bmps_enabled[3] << endl;
	cout << filters.update_temperature(temperatures) << endl;*/
}


