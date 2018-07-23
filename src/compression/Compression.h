#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cassert>
#include <string.h>
#include <unordered_map>

using namespace std;

//#define TIMING

#include "freqs.h"
#include "arith.h"

//#define DEBUG
//#define MKASM

#ifdef DEBUG
#define DD(x) cout << x;
#else
#define DD(x) {}
#endif

#ifdef MKASM
#define AD(x) cout << x << endl;
#else
#define AD(x) {}
#endif


class Jankompress {
public:
	Jankompress() {};

	void input(uint32_t time, float altitude);
	int output(uint8_t* arr, int max, float alt0);
	void comm_fail();
	void comm_success();
	uint32_t last = 0;
	void decode_block(float *block, float alt0, float scale, vector<float>& out);
	const uint32_t dt = 10000;
  int max_bytes = 20;

private:
	int next(int idx);
	int prev(int idx);
	void rescale();

	bool encode(int q, float alt0, buffer_t& enc);

	static const int N = 7200;
	float buffer[N] = {0};

	const int BUF_SIZE = 50;
	const int Qbit = 7;

	int current = 0;
	int last_comm = N-1;
	int comm_start = -1;
	int curcnt = 0;
  float scale;

	Jankompress(const Jankompress& other) = delete; // thank mr cain
	void operator=(const Jankompress& rhs) = delete;
};

#endif
