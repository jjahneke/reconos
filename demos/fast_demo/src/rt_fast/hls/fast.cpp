#include "reconos_thread.h"
#include "reconos_calls.h"

#include "fast_consts.h"

#define BASETYPE uint64_t
#define BYTES 8
#define MASK 7
#define DWORDS_KPT 7
#define DONEFLAG 0xffffffffffffffff

#define MAX_W 640 // In pixel
#define BYTEPERPIXEL 3
#define CC_W (MAX_W*BYTEPERPIXEL) // In Byte
#define CC_H 480
#define MAX_W_DEPTH (2*MAX_W)
#define CC_W_DEPTH (MAX_W/4)
#define MBF 40
#define FAST_TH 7
#define MAXPERBLOCK 100

const BASETYPE BYTEMASK = 0xff;
const BASETYPE SHORTMASK = 0xffff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAT_SIZE 48
#define MAT_SIZE_ANGLE 64
#define CACHE_LINES (WINDOW_SIZE * 2)

#if 0
	#define NPPC XF_NPPC1
	#define TYPEWIDTH 8
	#define TYPEDIV 1
	#define TYPESHIFT 0
	#define ROI_ROW_LO 8
	#define ROI_ROW_HI 39
	#define ROI_COL_LO 8
	#define ROI_COL_HI 39
#else
	#define NPPC XF_NPPC8
	#define TYPEWIDTH 64
	#define TYPEDIV 8
	#define TYPESHIFT 3
	#define ROI_ROW_LO 8
	#define ROI_ROW_HI 39
	#define ROI_COL_LO 1
	#define ROI_COL_HI 4
#endif

#if 1
uint8_t kernel(uint8_t b, uint8_t g, uint8_t r) {
	#pragma HLS inline
	return (uint8_t)(0.114*b + 0.587*g + 0.299*r);
}
#else
uint8_t kernel(uint8_t b, uint8_t g, uint8_t r) {
	#pragma HLS inline
	return (b + g + r)/3;
}
#endif

/* Converts ap_uint<W> to Q representation */
template <int Wi, int Wo>
ap_int<Wo> uint2q(ap_uint<Wi> v, uint8_t frac) {
	#pragma HLS inline
	return (v << frac); // == v * (2 << (frac-1) == v * 2**frac
}

/* Converts ap_int<W> to Q representation */
template <int Wi, int Wo>
ap_int<Wo> int2q(ap_int<Wi> v, uint8_t frac) {
	#pragma HLS inline
	return (v << frac); // == v * (2 << (frac-1) == v * 2**frac
}

/* Converts float to Q representation */
int16_t float2q(float v, uint8_t frac) {
	#pragma HLS inline
	return rint(v * (2 << (frac-1))); // == v * (2 << (frac-1) == v * 2**frac
}

void read_next_batch(hls::stream<uint64_t>& memif_hwt2mem, hls::stream<uint64_t>& memif_mem2hwt, BASETYPE ptr_d, BASETYPE ptr_i, ap_uint<64>* cacheDepth, ap_uint<TYPEWIDTH>* cache, BASETYPE _img_w, BASETYPE img_h, uint16_t& row_count) {
	BASETYPE _in[CC_W/BYTES + 1];
	#pragma HLS array_partition variable=_in complete
Loop_batchReadRow:
	for(int _row = 0; _row < WINDOW_SIZE; _row++) {
		BASETYPE ptr_limit = row_count % img_h;
		/*Read depth values*/
		BASETYPE _offsetD = ((ptr_d + ptr_limit * MAX_W_DEPTH) & MASK);
		BASETYPE _lenD = (MAX_W_DEPTH + _offsetD + BYTES)&(~MASK);
		BASETYPE _addrD = (ptr_d + ptr_limit * MAX_W_DEPTH)&(~MASK);
		MEM_READ(_addrD, &cacheDepth[CC_W_DEPTH * (row_count % CACHE_LINES)], _lenD);
		/*Read RGB values*/
		BASETYPE _offset = ((ptr_i + ptr_limit * _img_w) & MASK);
		BASETYPE _len = (_img_w + _offset + BYTES)&(~MASK);
		BASETYPE _addr = (ptr_i + ptr_limit * _img_w)&(~MASK);
		MEM_READ(_addr, &_in[0], _len);
#if 0
Loop_batchReadCol:
		for(ap_uint<10> ii = 0; ii < MAX_W; ii+=8) {
			ap_uint<64> _v;
			uint16_t _dw0_ptr = (_offset + ii*BYTEPERPIXEL) / BYTES;
			ap_uint<256> _buf;
			_buf(256,192) = _in[_dw0_ptr];
			_buf(192,128) = _in[_dw0_ptr + 1];
			_buf(128,64) = _in[_dw0_ptr + 2];
			_buf(64,0) = _in[_dw0_ptr + 3]; // Only overlapping read between iterations happens here

			uint8_t first_bidw = (_offset + ii*BYTEPERPIXEL) % BYTES; // "Pointer" to first blue channel of first pixel
			for(ap_uint<5> bgr = 0; bgr < 24; bgr+=3) {
				#pragma HLS unroll
				uint8_t _hi = 256-(first_bidw*8);
				uint8_t _lo = 248-(first_bidw*8);
				uint8_t _b = _buf(_hi-8*(bgr+0), _lo-8*(bgr+0));
				uint8_t _r = _buf(_hi-8*(bgr+1), _lo-8*(bgr+1));
				uint8_t _g = _buf(_hi-8*(bgr+2), _lo-8*(bgr+2));
				_v |= (ap_uint<64>)kernel(_b, _g, _r) << (bgr/3)*8;
			}
			cache[MAX_W/8 * (row_count % CACHE_LINES) + ii/8] = _v;
		}
#else
Loop_batchReadCol:
		for(int ii = 0; ii < MAX_W/8; ii++) {
			#pragma HLS pipeline
			ap_uint<64> _v;
			for(int b = 0; b < 8; b++) {
				uint16_t _dword_ptr = (_offset + ii*8 + BYTEPERPIXEL*b) / BYTES;
				uint8_t _bidw = (_offset + ii*8 + BYTEPERPIXEL*b) % BYTES;
				BASETYPE _dword0 = _in[_dword_ptr];
				BASETYPE _dword1 = _in[_dword_ptr+1];
				uint8_t _b = ((_dword0 & (BYTEMASK << _bidw*8)) >> _bidw*8);
				uint8_t _g = _bidw < 7 ? ((_dword0 & (BYTEMASK << (_bidw+1)*8)) >> (_bidw+1)*8) : _dword1 & BYTEMASK;
				uint8_t _r = _bidw < 6 ? ((_dword0 & (BYTEMASK << (_bidw+2)*8)) >> (_bidw+2)*8) : ((_dword1 & (BYTEMASK << (_bidw%6)*8)) >> (_bidw%6)*8);
				_v |= (ap_uint<64>)kernel(_b, _g, _r) << b*8;
			}
			cache[MAX_W/8 * (row_count % CACHE_LINES) + ii] = _v;
		}
#endif
		row_count++;
	}
}

typedef struct {
	ap_uint<10> global_x, global_y;
	ap_uint<10> angle_x, angle_y;
	uint8_t r;
	int16_t angle; // NOTE_J: How to handle this one?
	ap_int<18> xU, yU, xR;// Values in Q18.12.6
	uint16_t depth;
	ap_uint<1> done;
	ap_uint<10> startRow, startCol;
	ap_uint<64> desc[4];
} kpt_t;

void populate_xfMat(ap_uint<TYPEWIDTH>* cache, xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC>& _dst, hls::stream<ap_uint<64>>& _dstAngle, xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC>& _dstBlur, ap_uint<10> startRow, ap_uint<10> startCol) {
#if 0
Loop_FillRow:
	for(ap_int<8> _row = -1; _row < MAT_SIZE - 1; _row++) {
Loop_FillCol:
		for(ap_int<4> _col = -1; _col < MAT_SIZE/8 - 1; _col++) {
			#pragma HLS pipeline
			ap_uint<64> v = cache[((startRow + _row)%CACHE_LINES)*MAX_W/8 + (startCol/8 + _col)];
			_dst.write((_row+1) * MAT_SIZE/8 + (_col+1), v);
		}
	}

Loop_FillRowAB:
	for(ap_int<8> _row = -2; _row < MAT_SIZE_ANGLE - 2; _row++) {
Loop_FillColAB:
		for(ap_int<4> _col = -2; _col < MAT_SIZE_ANGLE/8 - 2; _col++) {
			#pragma HLS pipeline
			ap_uint<64> v = cache[((startRow + _row)%CACHE_LINES)*MAX_W/8 + (startCol/8 + _col)];
			_dstAngle.write(v);
			_dstBlur.write((_row+2) * MAT_SIZE_ANGLE/8 + (_col+2), v);
		}
	}
#else
Loop_FillRowMerged:
	for(ap_int<8> _row = -2; _row < MAT_SIZE_ANGLE - 2; _row++) {
Loop_FillColMerged:
		for(ap_int<4> _col = -2; _col < MAT_SIZE_ANGLE/8 - 2; _col++) {
			#pragma HLS pipeline
			ap_uint<64> v = cache[((startRow + _row)%CACHE_LINES)*MAX_W/8 + (startCol/8 + _col)];
			_dstAngle.write(v);
			_dstBlur.write((_row+2) * MAT_SIZE_ANGLE/8 + (_col+2), v);
			if(_row > -2 && _row < MAT_SIZE-1 && _col > -2 && _col < MAT_SIZE/8-1) {
				_dst.write((_row+1) * MAT_SIZE/8 + (_col+1), v);
			}
		}
	}
#endif
}

void evaluateFast(xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC>& _src, hls::stream<kpt_t> &_dst, ap_uint<10> startRow, ap_uint<10> startCol) {
Loop_EvalRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
	Loop_EvalCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE/TYPEDIV; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt;
			ap_uint<TYPEWIDTH> _r = _src.read(_mrow * MAT_SIZE/TYPEDIV + _mcol);
			// NOTE_J: (b7, b6, ..., b1, b0) -> [b0, b1, ..., b6, b7]

Loop_evalCalc:
			for(ap_uint<4> pix = 0; pix < TYPEDIV; pix++) {
				kpt.r = _r.range((pix+1)*TYPEDIV, pix*TYPEDIV);
				ap_uint<8> resp = _r.range((pix+1)*TYPEDIV, pix*TYPEDIV);
				kpt.global_x = startCol - 8 + _mcol*TYPEDIV + pix;
				kpt.global_y = startRow - 8 + _mrow;
				kpt.angle_x = 8 + _mcol*TYPEDIV + pix;
				kpt.angle_y = 8 + _mrow;
				// NOTE_J: Set flag when on pixel in row 39, col 39, i.e, on 31,31 in 32x32 matrix
				kpt.done = _mrow == ROI_ROW_HI && _mcol == ROI_COL_HI && pix == TYPEDIV-1 ? 1 : 0;
	
				kpt.startRow = startRow;
				kpt.startCol = startCol;
				// NOTE_J: Only keep points inside 32x32 ROI
				if(_mrow >= ROI_ROW_LO && _mrow <= ROI_ROW_HI && _mcol >= ROI_COL_LO && _mcol <= ROI_COL_HI && resp > 0) {
					_dst.write(kpt);
				}
			}
		}
	}
}

void undistort(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst) {
#if 0
Loop_UndistortRow:
	for(uint8_t _mrow = 0; _mrow < WINDOW_SIZE; _mrow++) {
Loop_UndistortCol:
		for(uint8_t _mcol = 0; _mcol < WINDOW_SIZE; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt = _src.read();
			kpt.xU = uint2q<10,18>(kpt.global_x, 6) + undistortLookupX[kpt.startRow][kpt.startCol];
			kpt.yU = uint2q<10,18>(kpt.global_y, 6) + undistortLookupY[kpt.startRow][kpt.startCol];
			_dst.write(kpt);
		}
	}
#else
	kpt_t kpt;
While_undistort:
	do {
		kpt = _src.read();
		kpt.xU = uint2q<10,18>(kpt.global_x, 6) + undistortLookupX[kpt.startRow][kpt.startCol];
		kpt.yU = uint2q<10,18>(kpt.global_y, 6) + undistortLookupY[kpt.startRow][kpt.startCol];
		_dst.write(kpt);
	}
	while(kpt.done != 1);
#endif
}

void computeStereo(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, ap_uint<64>* cacheDepth) {
#if 0
Loop_stereoRow:
	for(uint8_t _mrow = 0; _mrow < WINDOW_SIZE; _mrow++) {
Loop_stereoCol:
		for(uint8_t _mcol = 0; _mcol < WINDOW_SIZE; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt = _src.read();
			ap_uint<10> x = kpt.global_x;
			ap_uint<10> y = kpt.global_y;
			uint16_t _addr = y*CC_W_DEPTH + (x/4); // x/4 to find dword
			ap_uint<3> _sidw = x % 4;
			uint16_t d = (uint16_t)(cacheDepth[_addr] & (SHORTMASK << _sidw*16) >> _sidw*16);

			kpt.depth = d; // Filter for kpt.d == -1
			if(d > 0)
				kpt.xR = kpt.xU - float2q(MBF/d, 6);
			else
				kpt.xR = -64; // == -1 when converted back
			_dst.write(kpt);
		}
	}
#else
	kpt_t kpt;
While_computeStereo:
	do {
		kpt = _src.read();
		ap_uint<10> x = kpt.global_x;
		ap_uint<10> y = kpt.global_y;
		uint16_t _addr = y*CC_W_DEPTH + (x/4); // x/4 to find dword
		ap_uint<2> _sidw = x % 4;
		uint16_t d = (uint16_t)(cacheDepth[_addr] & (SHORTMASK << _sidw*16) >> _sidw*16);

		kpt.depth = d; // Filter for kpt.d == 0
		if(d > 0)
			kpt.xR = kpt.xU - float2q(MBF/d, 6);
		else
			kpt.xR = -1;
		_dst.write(kpt);
    }
	while(kpt.done != 1);
#endif
}

void calcAngle(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, hls::stream<ap_uint<TYPEWIDTH>>& _srcAngle) {
	const uint8_t u_max[16] = {15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3};
	#pragma HLS array_partition variable=u_max complete
	//const ap_uint<4> umax[31] = {3, 6, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3};

	ap_uint<8> mAngle[MAT_SIZE_ANGLE * MAT_SIZE_ANGLE];
	#pragma HLS array_partition variable=mAngle cyclic factor=64
Loop_fillAngleRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE_ANGLE; _mrow++) {
Loop_fillAngleCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE_ANGLE/8; _mcol++) {
			#pragma HLS pipeline
			ap_uint<TYPEWIDTH> _v = _srcAngle.read();
			for(ap_uint<4> b = 0; b < TYPEDIV; b++) {
				uint8_t v = _v.range((b+1)*TYPEDIV,b*TYPEDIV);
				mAngle[_mrow*MAT_SIZE_ANGLE + _mcol*TYPEDIV + b] = v;
			}
		}
	}


#if 0
Loop_angleRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_angleCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			kpt_t kpt = _src.read();
			ap_uint<10> row = kpt.angle_y;
			ap_uint<10> col = kpt.angle_x;
		
			ap_int<24> m_01, m_10;
Loop_calcRow:
			for(ap_uint<5> _row = 0; _row <= 15; _row++) {
Loop_calcCol:
				for(ap_int<6> _col = -15; _col <= 15; _col++) {
					#pragma HLS pipeline
					if(_col >= -u_max[_row] && _col <= u_max[_row]) {
						uint8_t val_plus = mAngle[(row + _row) * MAT_SIZE_ANGLE + (col + _col)];
						uint8_t val_minus = mAngle[(row - _row) * MAT_SIZE_ANGLE + (col + _col)];
						if(_row == 0) {
							m_10 += _col * val_minus;
						}
						else {
							m_10 += _col * (val_plus + val_minus);
							m_01 += _row * (val_plus - val_minus);
						}
					}
				}
			}
			kpt.angle = (XF_PI_FIXED + xf::cv::Atan2LookupFP24(m_01, m_10, 24, 0, 24, 0));
			_dst.write(kpt);
		}
	}

#else
	kpt_t kpt;
While_calcAngle:
	do {
		kpt = _src.read();
		ap_uint<10> row = kpt.angle_y;
		ap_uint<10> col = kpt.angle_x;
		
		ap_int<24> m_01, m_10;
Loop_calcRow:
		for(ap_uint<5> _row = 0; _row <= 15; _row++) {
Loop_calcCol:
			for(ap_int<6> _col = -15; _col <= 15; _col++) {
				#pragma HLS pipeline
				if(_col >= -u_max[_row] && _col <= u_max[_row]) {
					uint8_t val_plus = mAngle[(row + _row) * MAT_SIZE_ANGLE + (col + _col)];
					uint8_t val_minus = mAngle[(row - _row) * MAT_SIZE_ANGLE + (col + _col)];
					if(_row == 0) {
						m_10 += _col * val_minus;
					}
					else {
						m_10 += _col * (val_plus + val_minus);
						m_01 += _row * (val_plus - val_minus);
					}
				}
			}
		}
		kpt.angle = (XF_PI_FIXED + xf::cv::Atan2LookupFP24(m_01, m_10, 24, 0, 24, 0));
		_dst.write(kpt);
	} while(kpt.done != 1);
#endif
}

void calcDescriptor(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC>& _srcBlur) {
	uint8_t mDesc[MAT_SIZE_ANGLE * MAT_SIZE_ANGLE];
	//#pragma HLS array_partition variable=mAngle cyclic factor=64
Loop_fillDescRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE_ANGLE; _mrow++) {
Loop_fillDescCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE_ANGLE/8; _mcol++) {
			#pragma HLS pipeline
			ap_uint<64> _v = _srcBlur.read(_mrow * MAT_SIZE_ANGLE/8 + _mcol);
Loop_unpackPixels:
			for(ap_uint<4> b = 0; b < 8; b++) {
				uint8_t v = _v.range((b+1)*8,b*8);
				mDesc[_mrow*MAT_SIZE_ANGLE + _mcol*8 + b] = v;
			}
		}
	}
#if 0
Loop_descRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_descCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt = _src.read();

			ap_uint<10> x = kpt.angle_x;
			ap_uint<10> y = kpt.angle_y;
			for(ap_uint<9> bit = 0; bit < 256; bit++) {
				desc_bit_t pair = pattern[bit];
				kpt.descriptor.range(bit+1,bit) = mDesc[(y+pair.p1y)*MAT_SIZE_ANGLE + (x+pair.p1x)] < mDesc[(y+pair.p2y)*MAT_SIZE_ANGLE + (x+pair.p2x)] ? 1 : 0;
			}
			_dst.write(kpt);
		}
	}
#else
	kpt_t kpt;
While_calcDesc:
	do {
		kpt = _src.read();
		ap_uint<10> x = kpt.angle_x;
		ap_uint<10> y = kpt.angle_y;
		for(ap_uint<3> dw = 0; dw < 4; dw++) {
			#pragma HLS pipeline
			for(ap_uint<9> bit = 0; bit < 64; bit++) {
				desc_bit_t pair = pattern[dw*4 + bit];
				kpt.desc[dw].range(bit+1,bit) = mDesc[(y+pair.p1y)*MAT_SIZE_ANGLE + (x+pair.p1x)] < mDesc[(y+pair.p2y)*MAT_SIZE_ANGLE + (x+pair.p2x)] ? 1 : 0;
			}
		}
		_dst.write(kpt);
	} while(kpt.done != 1);
#endif
}

/**
[kp.x, kp.y ; depth, r] [kpU.X ; kpU.y] [kpR ; a] // If a is converted float on FPGA
[kp.x, kp.y ; depth, r] [kpU.X ; kpU.y] [kpR ; 0, a] // If a is converted to float on CPU
[kp.x, kp.y ; a, r] [kpU.X, kpU.y ; kpR, depth] // If all ap_fixed are converted to float on CPU
*/
void fillMem(hls::stream<kpt_t>& _src, BASETYPE* memOut) {
	ap_uint<10> read_index = 0;

#if 0
Loop_fillMemRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_fillMemCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt = _src.read();
			if(kpt.r > 0) {
				// To not transfer negative numbers, add 512 constant to ap_int<18> candidates
				uint64_t dword0 = ((uint64_t)kpt.global_x << 48) | ((uint64_t)kpt.global_y << 32) | ((uint64_t)kpt.depth << 16) | kpt.r;
				uint64_t dword1 = ((uint64_t)(kpt.xU+512) << 32) | (kpt.yU+512);
				uint64_t dword2 = ((uint64_t)(kpt.xR+512) << 32) | kpt.angle;
				memOut[(read_index+1)*7 - 6] = dword0;
				memOut[(read_index+1)*7 - 5] = dword1;
				memOut[(read_index+1)*7 - 4] = dword2;
				memOut[(read_index+1)*7 - 3] = kpt.desc[3];
				memOut[(read_index+1)*7 - 2] = kpt.desc[2];
				memOut[(read_index+1)*7 - 1] = kpt.desc[1];
				memOut[(read_index+1)*7 - 0] = kpt.desc[0];
				read_index++;
			}
		}
	}
	memOut[0] = read_index;


#else
	kpt_t kpt;
While_fillMem:
	do {
		kpt = _src.read();
		// To not transfer negative numbers, add 512 constant to ap_int<18> candidates
		memOut[(read_index+1)*7 - 6] = ((uint64_t)kpt.global_x << 48) | ((uint64_t)kpt.global_y << 32) | ((uint64_t)kpt.depth << 16) | kpt.r;
		memOut[(read_index+1)*7 - 5] = ((uint64_t)(kpt.xU+512) << 32) | (kpt.yU+512);
		memOut[(read_index+1)*7 - 4] = ((uint64_t)(kpt.xR+512) << 32) | kpt.angle;
		memOut[(read_index+1)*7 - 3] = kpt.desc[3];
		memOut[(read_index+1)*7 - 2] = kpt.desc[2];
		memOut[(read_index+1)*7 - 1] = kpt.desc[1];
		memOut[(read_index+1)*7 - 0] = kpt.desc[0];
		read_index++;
	} while(kpt.done != 1);
	memOut[0] = read_index;
#endif
}

void dataflow_region(ap_uint<TYPEWIDTH>* cache, ap_uint<64>* cacheDepth, BASETYPE* memOut, ap_uint<10> startRow, ap_uint<10> startCol) {
	
	xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC> mFast_in(MAT_SIZE, MAT_SIZE);
	xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC> mFast_out(MAT_SIZE, MAT_SIZE);
	
	xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC> mBlur_in(MAT_SIZE_ANGLE, MAT_SIZE_ANGLE);
	xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC> mBlur_out(MAT_SIZE_ANGLE, MAT_SIZE_ANGLE);
	
	hls::stream<kpt_t> strm_eval2Undistort;
	hls::stream<kpt_t> strm_undistort2Stereo;
	hls::stream<kpt_t> strm_stereo2Angle;
	hls::stream<kpt_t> strm_angle2Desc;
	hls::stream<kpt_t> strm_desc2Mem;
	hls::stream<ap_uint<TYPEWIDTH>> mAngle;
	#pragma HLS stream variable=strm_eval2Undistort depth=32
	#pragma HLS stream variable=strm_undistort2Stereo depth=32
	#pragma HLS stream variable=strm_stereo2Angle depth=32
	#pragma HLS stream variable=strm_angle2Desc depth=32
	#pragma HLS stream variable=strm_desc2Mem depth=32

	{
		#pragma HLS dataflow
		populate_xfMat(cache, mFast_in, mAngle, mBlur_in, startRow, startCol);
		xf::cv::fast<1, XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC>(mFast_in, mFast_out, FAST_TH);
		xf::cv::GaussianBlur<5,XF_BORDER_CONSTANT,XF_8UC1,MAT_SIZE_ANGLE,MAT_SIZE_ANGLE,NPPC>(mBlur_in, mBlur_out, 2);
		evaluateFast(mFast_out, strm_eval2Undistort, startRow, startCol);
		undistort(strm_eval2Undistort, strm_undistort2Stereo);
		computeStereo(strm_undistort2Stereo, strm_stereo2Angle, cacheDepth);
		calcAngle(strm_stereo2Angle, strm_angle2Desc, mAngle);
		calcDescriptor(strm_angle2Desc, strm_desc2Mem, mBlur_out);
		fillMem(strm_desc2Mem, memOut);
	}
}

THREAD_ENTRY() {
	THREAD_INIT();

/* These are pragmas to be applied to variables in the header file */
//	#pragma HLS data_pack variable=pattern[0]
//	#pragma HLS data_pack variable=pattern[1]

	ap_uint<TYPEWIDTH> cache[MAX_W/TYPEDIV * CACHE_LINES];
	//#pragma HLS array_partition variable=cache cyclic factor=640
	uint16_t row_count = 0;
	ap_uint<64> cacheDepth[MAX_W * CACHE_LINES];
	BASETYPE memOut[DWORDS_KPT*MAXPERBLOCK];
	#pragma HLS array_partition variable=memOut cyclic factor=7

	BASETYPE ptr_i = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_d = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE _img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_h = MBOX_GET(rcsfast_sw2rt);

	ap_uint<6> NROWS = img_h == CC_H ? (img_h - 2*BORDER_EDGE) / WINDOW_SIZE : 1 + (img_h - 2*BORDER_EDGE) / WINDOW_SIZE;
	ap_uint<6> NCOLS = img_w == MAX_W ? (img_w - 2*BORDER_EDGE) / WINDOW_SIZE : 1 + (img_w - 2*BORDER_EDGE) / WINDOW_SIZE;

	read_next_batch(memif_hwt2mem, memif_mem2hwt, ptr_d, ptr_i, &cacheDepth[0], &cache[0], _img_w, img_h, row_count);
	MBOX_PUT(rcsfast_rt2sw, 42);
Loop_RowStep:
	for(ap_uint<6> rowStep = 0; rowStep < NROWS; rowStep++) {
		MBOX_PUT(rcsfast_rt2sw, rowStep);
		read_next_batch(memif_hwt2mem, memif_mem2hwt, ptr_d, ptr_i, &cacheDepth[0], &cache[0], _img_w, img_h, row_count);

		ap_uint<10> startRow = BORDER_EDGE + rowStep*WINDOW_SIZE;
		//ap_uint<10> startRow = (BORDER_EDGE-3) + rowStep*WINDOW_SIZE;
		ap_uint<10> endRow = startRow + WINDOW_SIZE + 6;

Loop_ColStep:
		for(ap_uint<6> colStep = 0; colStep < NCOLS; colStep++) {
			MBOX_PUT(rcsfast_rt2sw, rowStep*NROWS+colStep);
			ap_uint<10> startCol = BORDER_EDGE + colStep*WINDOW_SIZE;
			//ap_uint<10> startCol = (BORDER_EDGE-3) + colStep*WINDOW_SIZE;
			ap_uint<10> endCol = startCol + WINDOW_SIZE + 6;
	

			{ // Region 1
				dataflow_region(&cache[0], &cacheDepth[0], &memOut[0], startRow, startCol);
			}

			BASETYPE _wroffset = MAXPERBLOCK * (rowStep*NCOLS + colStep);
			MEM_WRITE(&memOut[0], (ptr_o + (_wroffset*DWORDS_KPT*BYTES)), MAXPERBLOCK*DWORDS_KPT*BYTES);
		}
	}
	MBOX_PUT(rcsfast_rt2sw, DONEFLAG);
}
