#include "Config.h"

class Jankompress {
public:
	Jankompress(const uint16_t *huf, const uint8_t *len, const float *q, const int K, const int P) : huffman(huf), lengths(len), Q(q), K(K), P(P) {};

	int getCoefficientLength(int k, int nz, int s);
	int getCoefficient(int k, int nz, int s);
	inline int getIndex(int k, int nz, int s);

	void heresAnAltitudeKid(uint32_t time, float altitude);
	int giveMeBitsBitte(uint8_t* arr, int start, int max, float alt);
	void kommuniziert();
	uint32_t last = 0;

private:
	int next(int idx);
	int prev(int idx);

	void copyNumber(int number, int size, uint8_t *buffer, int& byteidx, int& bitidx);
	void encode(float* alts, uint8_t* enc, int& byteidx);
	const uint16_t *huffman;
	const uint8_t *lengths;
	const float *Q;
	const int K;
	const int P;
	static const int N = 500;
	float buffer[N] = {0};
	const uint32_t dt = 2000;
	//uint32_t times[N] = {0};

	int current = 0;
	int transmitted = -1;
	int curcnt = 0;

	Jankompress(const Jankompress& other) = delete;
	void operator=(const Jankompress& rhs) = delete;
};
