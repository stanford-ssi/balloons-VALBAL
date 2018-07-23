#ifndef ARITH_H

#define ARITH_H

// See: https://web.stanford.edu/class/ee398a/handouts/papers/WittenACM87ArithmCoding.pdf

#include <stdint.h>

typedef struct {
	uint8_t *buf;
	uint8_t bufs;
	uint8_t byteidx;
	int8_t bitidx;
} buffer_t;

inline bool copy_number(int number, int size, buffer_t *buffer) {
	if (buffer->byteidx >= buffer->bufs) return false;
	for (int j = size - 1; j >= 0; j--) {
		bool bit = number & (1 << j);
		if (bit) {buffer->buf[buffer->byteidx] |= (1 << buffer->bitidx);}
		buffer->bitidx -= 1;
		if (buffer->bitidx < 0) {
			buffer->bitidx = 7;
			buffer->byteidx += 1;
			if (buffer->byteidx >= buffer->bufs) return false;
		}
	}
	return true;
}

inline int read_bit(buffer_t *buffer) {
	if (buffer->byteidx >= buffer->bufs) return 0;
	int bit = (buffer->buf[buffer->byteidx] & (1 << buffer->bitidx)) != 0;
	buffer->bitidx--;
	if (buffer->bitidx < 0) {
		buffer->bitidx = 7;
		buffer->byteidx++;
	}
	return bit;
}


class ArithmeticCoder {
public:
	void start_encoding(buffer_t *buf_) {
		buf = buf_;
		low = 0;
		high = TOP;
		bits_to_follow = 0;
	}

	void start_decoding(buffer_t *buf_) {
		buf = buf_;
		low = 0;
		high = TOP;
		value = 0;
		for (int i = 1; i <= BITS; i++) {
			value = 2*value + next_bit();
		}
	}

	bool encode_symbol(int symbol, const uint16_t *cum_freq) {
		int32_t range = (high-low)+1;
		high = low + (range*cum_freq[symbol])/cum_freq[0] - 1;
		low = low + (range*cum_freq[symbol+1])/cum_freq[0];

		for (int maxiter=0; maxiter<16; maxiter++) {
			if (high < HALF) {
				if (!bit_plus_follow(0)) return false;
			} else if (low >= HALF) {
				if (!bit_plus_follow(1)) return false;
				low -= HALF;
				high -= HALF;
			} else if (low >= FIRST_QTR && high < THIRD_QTR) {
				bits_to_follow += 1;
				low -= FIRST_QTR;
				high -= FIRST_QTR;
			} else break;
			low = 2*low;
			high = 2*high + 1;
		}
		return true;
	}

	int decode_symbol(const uint16_t *cum_freq) {
		int32_t range = (high-low) + 1;
		int cum = (((value-low)+1)*cum_freq[0]-1)/range;

		int symbol;
		for (symbol=1; cum_freq[symbol] > cum; symbol++) {};

		high = low + (range*cum_freq[symbol-1])/cum_freq[0] - 1;
		low = low + (range*cum_freq[symbol])/cum_freq[0];

		for (int maxiter=0; maxiter<16; maxiter++) {
			if (high < HALF) {}
			else if (low >= HALF) {
				value -= HALF;
				low -= HALF;
				high -=  HALF;
			} else if (low >= FIRST_QTR && high < THIRD_QTR) {
				value -= FIRST_QTR;
				low -= FIRST_QTR;
				high -= FIRST_QTR;
			} else break;
			low = 2*low;
			high = 2*high+1;
			value = 2*value + next_bit();
		}
		return symbol - 1;
	}

	bool finish_encoding() {
		bits_to_follow++;
		if (low < FIRST_QTR) return bit_plus_follow(0);
		else return bit_plus_follow(1);
	}

private:
	int32_t low, high, bits_to_follow, value;
	const int BITS = 16;
	const int TOP = (1 << 16)-1;
	const int FIRST_QTR = TOP/4+1;
	const int HALF = 2*FIRST_QTR;
	const int THIRD_QTR = 3*FIRST_QTR;
	buffer_t *buf;

	bool bit_plus_follow(int bit) {
		if (!copy_number(bit, 1, buf)) return false;
		int max_bits = 64;
		while (bits_to_follow > 0 && max_bits > 0) {
			if (!copy_number(!bit, 1, buf)) return false;
			bits_to_follow--;
			max_bits--;
		}
		return true;
	}

	int next_bit() {
		return read_bit(buf);
	}
};

#endif
