#include "Compression.h"
#include "arith.h"

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

void Jankompress::rescale() {
}

void Jankompress::input(uint32_t time, float altitude) {
	if (last == 0) { // first loop
		last = ((time + (dt/2))/dt)*dt;
	} else {
		int niter = 0;
		while (time >= last + dt/2) {
			if (niter++ > 10) { // what even, there was a huge gap
				last = 0;
				current = 0;
				last_comm = N-1;
				curcnt = 0;
				break;
			}
			niter++;

			if (curcnt > 0) {
				buffer[current] /= curcnt;
			} else {
				buffer[current] = buffer[prev(current)];
			}
			curcnt = 0;
			current = next(current);
			last = last + dt;
			if (current == last_comm) {
        last_comm = (last_comm + DCT_N + 1) % N; // not my problem
			}
		}
		if (curcnt == 0) {
			buffer[current] = 0;
		}
	}
	buffer[current] += altitude;
	curcnt++;
}

void Jankompress::comm_success() {
	last_comm = comm_start;
}

int Jankompress::output(uint8_t* arr, int max, float alt0) {
	// push last reading onto buffer
    if (curcnt > 0) {
        buffer[current] /= curcnt;
    } else {
        buffer[current] = buffer[prev(current)];
    }
    curcnt = 0;
    last = last + dt;

	// store what's the last thing we sent in case it's successful
	comm_start = current;
  current = next(current);

	uint8_t buf[BUF_SIZE];

	buffer_t buff;
	buff.buf = buf;
	buff.bufs = BUF_SIZE;
	buff.byteidx = 3;
	buff.bitidx = 7;

  bool ok;
  int i;
  for (i=0; i<5; i++) {
    memset(buf, 0, BUF_SIZE);
    buff.byteidx = 3;
    buff.bitidx = 7;
    copy_number(i-1, 3, &buff);
    ok = encode(1<<i, alt0, buff);
    if (ok && buff.byteidx >= max) {
      ok = false;
    }
    if (ok) break;
  }
  if (buff.bitidx != 7) buff.byteidx++;
  uint16_t t = last - dt - (1 << i)*dt/2.;
  memcpy(buf, &t, 2);
  buf[2] = round(255 * log10(scale) / 4.);;

  memcpy(arr, buf, buff.byteidx);

}

bool Jankompress::encode(int Q, float alt0, buffer_t& buff) {
	int niter = 0;
	int pos = comm_start;
	// first pass: get slope, scale factor, number of blocks
	float mx = -1e6;
	float mn = 1e6;
	float ac;
	AD("start")
	while (niter++ <= N/(DCT_N*Q)) {
		bool stop = false;
		for (int i=0; i<DCT_N; i++) {
			ac = 0;
			for (int j=0; j<Q; j++) {
				ac += buffer[pos];
				pos = prev(pos);
				if (pos == last_comm) {
					stop = true;
				}
			}
			ac /= Q;
			AD("val " << ac)
			ac -= alt0;
			if (ac > mx) mx = ac;
			if (ac < mn) mn = ac;
		}
		if (stop) break;
	}
	scale = mx - mn;
	AD("alt0 " << alt0)
	AD("scale " << scale)
	AD("time " << last - dt - Q*dt/2)
	AD("q " << Q)

	ArithmeticCoder arith;
	arith.start_encoding(&buff);

	pos = comm_start;
	int prevDC = 0;
	int nb = 0;
	for (int b=0; b<niter; b++) {
		float block[DCT_N];
		for (int i=0; i<DCT_N; i++) {
			float ac = 0;
			for (int j=0; j<Q; j++) {
				ac += buffer[pos];
				pos = prev(pos);
			}
			ac /= Q;
			AD("# " << ac)
			ac -= alt0;
			block[i] = ac/scale * (pow(2, Qbit) - 1);
		}
		int nz = 0;
		int aci;
		for (int i=0; i<DCT_N; i++) {
			float s = 0;
			for (int j=0; j < DCT_N; j++) {
          		s += block[j] * DCT[j][i];
			}
			int trans = round(s/Qvec[i]);
			//cout << "# " << trans << endl;
			//float trans = s/Qvec[i];
			if (i == 0) {
				int delta = trans - prevDC;
				AD("dc " << delta)
				int sym = delta + DC_MAX;
				if (sym < 0) sym = 0;
				if (sym >= 2*DC_MAX) sym = 2*DC_MAX - 1;
				if (!arith.encode_symbol(sym, DC_FREQS)) return false;

				//cout << "# orig" << trans << endl;
				prevDC = trans;
				aci = 0;
			} else {
				if (trans == 0) nz++;
				else {
					AD("ac " << nz << " " << trans)
					int sym = nz + 1;
					if (!arith.encode_symbol(sym, NZ_FREQS[aci])) return false;
					int sym2 = trans;
					//if (sym2 == 0) { cout << "u w0t" << endl; exit(1); }
					if (sym2 > 0) sym2--;
					sym2 += AC_MAX;
					if (sym2 < 0) { sym2 = 0;}
					if (sym2 >= 2*AC_MAX - 1) { sym2 = 2*AC_MAX - 2; }
					if (!arith.encode_symbol(sym2, AC_FREQS)) return false;
					aci++;
					nz = 0;
				}
			}
			nb += nz;
		}
		if (nz != 0) {
			AD("eob")
			if (!arith.encode_symbol(0, NZ_FREQS[aci])) return false;
		}

        /*for (int n = 0; n < 16; n++) {
        }*/
	}
	if (!arith.finish_encoding()) return false;
	#ifdef MKASM
		cout << "binstat";
		int kk = 0;
		kk += 8*buff.byteidx;
		kk += (7-buff.bitidx);
		cout << " " << kk << endl;
		int plus = (buff.bitidx == 7) ? 0 : 1;
		int cnt = (int)(buff.byteidx+plus);
		cout << "bin "; cout << cnt;
		for (int i=0; i<cnt; i++) {
			cout << " " << (int)buf[i];
		}
		cout << endl;
	#endif
	//cout << prevDC << " " << nb << endl;
	AD("end")

  return true;
}

void Jankompress::decode_block(float *block, float alt0, float scale, vector<float>& out) {
	float alts[DCT_N] = {0};
	for (int i=0; i<DCT_N; i++) {
		block[i] *= Qvec[i]*2/DCT_N;
		alts[i] = 0.5 * block[0];
	}

	for (int i=0; i<DCT_N; i++) {
		for (int j=1; j<DCT_N; j++) {
			alts[i] += block[j]*DCT[i][j];
		}
	}
	for (int i=0; i<DCT_N; i++) {
		alts[i] *= scale/(pow(2, Qbit) - 1);
		alts[i] += alt0;
		out.push_back(alts[i]);
		//cout << "got " << alts[i] << endl;
	}
}
