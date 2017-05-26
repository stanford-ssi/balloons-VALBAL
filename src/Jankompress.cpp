#include "Jankompress.h"
#include "pca_data.h"

int Jankompress::next(int idx) {
	if (idx == N-1)
		return 0;
	return idx + 1;
}

int Jankompress::prev(int idx) {
	if (idx == 0)
		return N-1;
	return idx - 1;
}

void Jankompress::heresAnAltitudeKid(uint32_t time, float altitude) {
	if (current == transmitted) {
		transmitted = next(current);
	}
	if (last == 0) {
		last = ((time + (dt/2))/dt)*dt;
	} else {
		int niter = 0;
		while (time >= last + dt/2) {
			if (niter > 10) break; // what even
			niter++;

			if (curcnt > 0) {
				buffer[current] /= curcnt;
			} else {
				buffer[current] = buffer[prev(current)];
			}
			curcnt = 0;
			current = next(current);
			last = last + dt;
		}
		if (curcnt == 0) {
			buffer[current] = 0;
		}
	}
	buffer[current] += altitude;
	curcnt += 1;
}


inline int Jankompress::getIndex(int k, int nz, int s) {
	int start = 9*(23-k)*k/2;
	int zstart = (K-1)*nz;
	int idx = start+zstart+s;
	return idx;
}

int Jankompress::getCoefficient(int k, int nz, int s) {
	return huffman[getIndex(k, nz, s)];
}

int Jankompress::getCoefficientLength(int k, int nz, int s) {
	int idx = getIndex(k, nz, s);
	uint8_t val = lengths[idx/2];
	if (idx % 2 == 0)
		return ((val & 0xf0) >> 4)+1;
	else
		return (val & 0xf)+1;
}

void Jankompress::copyNumber(int number, int size, uint8_t *buffer, int& byteidx, int& bitidx) {
	if (byteidx > 45) return;
	for (int j = size - 1; j >= 0; j--) {
		bool bit = number & (1 << j);
		if (bit) {buffer[byteidx] |= (1 << bitidx);} //cout << "[1]";}
		//else { cout << "[0]";}
		bitidx -= 1;
		if (bitidx < 0) {
			bitidx = 7;
			byteidx += 1;
		}
	}
}


void Jankompress::encode(float* alts, uint8_t* enc, int& byteidx) {
	int bitidx = 7;
	float pca[K];
	for (int i=0; i<K; i++) {
		pca[i] = 0;
		for (int j=0; j<P; j++) {
			pca[i] += alts[j]*PCA_DATA[i][j];
		}
		//cout << "HEY " << pca[i] << endl;
		pca[i] /= Q[i];
	}
	int bigidx = 0;
	float bigmag = fabs(pca[0]);
	int sgn = pca[0] >= 0;
	for (int i=1; i<4; i++) {
		if (fabs(pca[i]) > bigmag) {
			bigidx = i;
			bigmag = fabs(pca[i]);
			sgn = pca[i] >= 0;
		}
	}
	//cout << "ref" << bigmag << endl;
	//cout << dec;
	//cout << "big idx " << bigidx << endl;
	copyNumber(bigidx, 2, enc, byteidx, bitidx);
	copyNumber(sgn, 1, enc, byteidx, bitidx);
	//cout << "(" << byteidx << "," << bitidx<<endl;
	uint32_t flt = *(uint32_t*)&bigmag; // evil floating point bit level hacking
	uint32_t exp = ((flt & 0x7f800000) >> 23)-127; // what the fuck?
	uint32_t bts = (flt & 0x007c0000) >> 18;
	//cout << "exp " << exp << endl;
	copyNumber(exp, 4, enc, byteidx, bitidx);
	//cout << "man " << bts << endl;
	copyNumber(bts, 5, enc, byteidx, bitidx);

	int nz = 0; int fz = 0; // casualty is important
	for (int i=0; i<K; i++) {
		if (i == bigidx) {
			fz = i+1;
			continue;
		}
		int ac = round(pca[i]*128./bigmag);
		int fac = (ac < 0) ? -ac : ac;
		if (ac == 0) {
			nz++;
		} else {
			int sz = floor(log2(fac)+1);
			//cout << "coef " << fz << "," << nz<<","<<sz<<"  "<<getCoefficientLength(fz,nz,sz)<<endl;
			copyNumber(getCoefficient(fz,nz,sz), getCoefficientLength(fz,nz,sz), enc, byteidx, bitidx);
			nz = 0;
			//cout << "number" << endl;
			if (ac >= 0) copyNumber(ac, sz, enc, byteidx, bitidx);
			else copyNumber(ac-1, sz, enc, byteidx, bitidx);
			fz = i+1;
		}
	}
	//fz = (fz < K) ? fz : (K-1);
	if (fz != 10) {
		//cout << "coef " << fz << "," << nz<<","<<0<<"  "<<getCoefficient(fz,nz,0)<<endl;
		copyNumber(getCoefficient(fz,nz,0), getCoefficientLength(fz,nz,0), enc, byteidx, bitidx);
	}
	if (bitidx != 7) byteidx++;
}

int Jankompress::giveMeBitsBitte(uint8_t* arr, int start, int max, float alt) {
	if (curcnt > 0) {
		buffer[current] /= curcnt;
	} else {
		buffer[current] = buffer[prev(current)];
	}
	current = next(current);
	curcnt = 0;
	last = last + dt;

	float alts[P];
	uint8_t encoded[50] = {0};
	int ptr = prev(current); // lel
	bool stahp = false;
	int byteidx = 0;
	/*for (int i=0; i<4; i++) {
		if (stahp) break;
		if (i !=0) { cout << "yo" << endl;}*/

		for (int k=P-1; k >= 0; k--) {
			alts[k] = buffer[ptr]-alt-MEANS[k];
			ptr = prev(ptr);
			if (ptr == transmitted) stahp = true;
		}
		int prevlen = byteidx;
		encode(alts, encoded, byteidx);
		if (byteidx > max) {
			byteidx = prevlen;
		}
	//}
  for (int i=start; i<(start+byteidx); i++) {
    arr[i] = encoded[i-start];
  }
	/*for (int b=0; b<byteidx; b++) {
			cout << setfill('0') << setw(2) << hex << (unsigned int) encoded[b];
	}
	cout << endl;*/
	return byteidx;
}

void Jankompress::kommuniziert() {
	transmitted = current;
}
