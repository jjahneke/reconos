#include "reconos_thread.h"
#include "reconos_calls.h"

#include "common/xf_common.hpp"
#include "core/xf_math.h"
#include "features/xf_fast.hpp"
#include "hls_math.h"
#include "ap_utils.h"

#define BASETYPE uint64_t
#define BYTES 8
#define MASK 7
#define DWORDS_KPT 3
#define DONEFLAG 0xffffffffffffffff

#define MAX_W 640 // In pixel
#define BYTEPERPIXEL 3
#define CC_W (MAX_W*BYTEPERPIXEL) // In Byte
#define CC_H 480
#define MAX_W_DEPTH (2*MAX_W)
#define CC_W_DEPTH (MAX_W/4)
#define MBF 40
const BASETYPE BYTEMASK = 0xff;
const BASETYPE SHORTMASK = 0xffff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAT_SIZE (WINDOW_SIZE + 6)
#define CACHE_LINES (WINDOW_SIZE * 2)
#define MAT_SIZE_ANGLE (WINDOW_SIZE + 30)

#define FAST_TH 7
#define MAXPERBLOCK 100

uint8_t kernel(uint8_t b, uint8_t g, uint8_t r) {
	#pragma HLS inline
	return (uint8_t)(0.114*b + 0.587*g + 0.299*r);
}

/* Converts ap_uint<W> to Q representation */
template <int Wi, int Wo>
ap_int<Wo> uint2q(ap_uint<Wi> v, uint8_t frac) {
	#pragma HLS inline
	return (v << frac); // == v * (2 << (frac-1) == v * 2**frac
}

/* Converts float to Q representation */
int16_t float2q(float v, uint8_t frac) {
	#pragma HLS inline
	return rint(v * (2 << (frac-1))); // == v * (2 << (frac-1) == v * 2**frac
}

static uint16_t row_count = 0;
void read_next_batch(hls::stream<uint64_t>& memif_hwt2mem, hls::stream<uint64_t>& memif_mem2hwt, BASETYPE ptr_d, BASETYPE ptr_i, BASETYPE* cacheDepth, uint8_t* cache, BASETYPE _img_w, BASETYPE img_h) {
	BASETYPE _in[CC_W/BYTES + 1];
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
		for(int ii = 0; ii < MAX_W; ii++) {
			uint16_t _dword_ptr = (uint16_t)((_offset + ii*BYTEPERPIXEL) / BYTES);
			uint8_t _bidw = (uint8_t)((_offset + ii*BYTEPERPIXEL) % BYTES);
			BASETYPE _dword0 = _in[_dword_ptr];
			BASETYPE _dword1 = _in[_dword_ptr+1];
			uint8_t _b = ((_dword0 & (BYTEMASK << _bidw*8)) >> _bidw*8);
			uint8_t _g = _bidw < 7 ? ((_dword0 & (BYTEMASK << (_bidw+1)*8)) >> (_bidw+1)*8) : _dword1 & BYTEMASK;
			uint8_t _r = _bidw < 6 ? ((_dword0 & (BYTEMASK << (_bidw+2)*8)) >> (_bidw+2)*8) : ((_dword1 & (BYTEMASK << (_bidw%6)*8)) >> (_bidw%6)*8);
			BASETYPE _cache_line = MAX_W * (row_count % CACHE_LINES);
			cache[_cache_line + ii] = kernel(_b, _g, _r);
		}
		row_count++;
	}
}


/* This is for TUM1, fixed point 16 bit, 10 integer, 6 fractional */
const ap_int<11> undistortLookupX[14][19] = {
{ -576, -493, -444, -416, -398, -385, -373, -361, -350, -339, -330, -326, -336, -370, -446, -580, -786, -1083, -1406},
{ -434, -394, -375, -364, -354, -345, -334, -324, -314, -304, -293, -282, -274, -276, -304, -375, -508, -717, -935},
{ -352, -337, -329, -321, -310, -298, -286, -276, -268, -263, -257, -250, -240, -230, -231, -257, -332, -472, -628},
{ -296, -290, -283, -271, -255, -239, -224, -214, -209, -209, -210, -210, -207, -198, -189, -192, -225, -310, -418},
{ -247, -242, -231, -214, -194, -173, -157, -147, -144, -148, -156, -164, -168, -166, -157, -151, -159, -205, -275},
{ -197, -190, -176, -156, -133, -112, -95, -86, -85, -92, -103, -116, -126, -129, -125, -117, -115, -135, -177},
{ -144, -137, -123, -103, -82, -62, -47, -39, -40, -47, -60, -74, -85, -92, -91, -85, -79, -85, -107},
{ -91, -85, -75, -60, -44, -29, -18, -12, -13, -19, -29, -40, -50, -56, -56, -51, -44, -43, -53},
{ -38, -36, -32, -25, -17, -10, -4, -1, -2, -5, -10, -15, -19, -21, -20, -16, -9, -5, -6},
{ 17, 13, 9, 6, 4, 2, 1, 0, 0, 2, 3, 6, 9, 13, 17, 21, 27, 33, 38},
{ 73, 63, 51, 39, 26, 15, 8, 4, 5, 10, 19, 29, 39, 48, 55, 60, 65, 73, 84},
{ 132, 118, 100, 80, 59, 41, 28, 21, 22, 31, 44, 60, 75, 88, 97, 102, 106, 117, 135},
{ 194, 179, 158, 133, 108, 85, 68, 59, 60, 69, 85, 103, 121, 134, 142, 145, 149, 167, 198},
{ 243, 228, 208, 182, 155, 131, 112, 103, 103, 112, 127, 145, 161, 172, 178, 179, 186, 213, 257}};

/* This is for TUM1, fixed point 16 bit, 10 integer, 6 fractional */
const ap_int<11> undistortLookupY[14][19] = {
{ -559, -425, -332, -259, -199, -145, -96, -51, -10, 27, 62, 97, 138, 198, 298, 467, 740, 1161, 1639},
{ -481, -382, -308, -245, -188, -135, -85, -40, 1, 38, 72, 103, 132, 168, 226, 335, 534, 862, 1226},
{ -444, -365, -298, -236, -178, -124, -75, -31, 9, 46, 81, 113, 141, 167, 200, 264, 398, 650, 945},
{ -430, -358, -291, -225, -164, -109, -63, -23, 13, 49, 84, 119, 150, 176, 199, 235, 320, 505, 742},
{ -425, -352, -279, -208, -145, -92, -50, -16, 14, 45, 79, 116, 153, 184, 207, 230, 281, 412, 600},
{ -422, -343, -263, -187, -123, -72, -36, -10, 12, 36, 68, 107, 149, 187, 215, 236, 268, 359, 505},
{ -418, -332, -246, -166, -100, -53, -24, -6, 8, 26, 54, 94, 140, 185, 221, 244, 267, 331, 448},
{ -414, -322, -230, -147, -82, -37, -13, -3, 4, 16, 42, 81, 131, 182, 224, 251, 271, 320, 416},
{ -412, -316, -221, -135, -70, -27, -7, -1, 1, 10, 33, 73, 125, 180, 227, 257, 277, 319, 403},
{ -414, -317, -219, -133, -67, -25, -5, 0, 0, 8, 31, 72, 125, 181, 230, 262, 283, 323, 403},
{ -421, -324, -227, -141, -74, -30, -8, -1, 2, 13, 37, 78, 131, 188, 235, 267, 288, 331, 415},
{ -431, -337, -243, -158, -90, -43, -16, -3, 7, 22, 50, 92, 144, 197, 242, 271, 294, 345, 441},
{ -443, -354, -264, -181, -113, -62, -28, -6, 12, 34, 66, 109, 159, 208, 248, 274, 301, 368, 485},
{ -453, -368, -282, -202, -132, -78, -38, -9, 16, 44, 80, 123, 171, 216, 251, 276, 309, 395, 533}};


typedef struct {
	ap_uint<10> global_x, global_y;
	ap_uint<10> angle_x, angle_y;
	uint8_t r;
	int16_t angle; // NOTE_J: How to handle this one?
	ap_int<18> xU, yU, xR;// Values in Q18.12.6
	uint16_t depth;
	ap_uint<1> done;
	ap_uint<10> startRow, startCol;
} kpt_t;

void populate_xfMat(uint8_t* cache, hls::stream<uint8_t>& mAngle, xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1>& _dst, ap_uint<10> startRow, ap_uint<10> startCol) {
#if 0
Loop_FillRow:
	for(uint8_t _row = 0; _row < MAT_SIZE; _row++) {
Loop_FillCol:
		for(uint8_t _col = 0; _col < MAT_SIZE; _col++) {
			#pragma HLS pipeline
			uint8_t v = cache[(startRow+_row)%CACHE_LINES * MAX_W + (startCol+_col)];
			_dst.write(_row * MAT_SIZE + _col, v);
		}
	}

	ap_uint<10> angleRow = startRow - 15;
	ap_uint<10> angleCol = startCol - 15;
Loop_FillRowA:
	for(uint8_t _row = 0; _row < MAT_SIZE_ANGLE; _row++) {
Loop_FillColA:
		for(uint8_t _col = 0; _col < MAT_SIZE_ANGLE; _col++) {
			#pragma HLS pipeline
			uint8_t v = cache[(angleRow+_row)%CACHE_LINES * MAX_W + (angleCol+_col)];
			mAngle[_row * MAT_SIZE_ANGLE + _col] = v;
		}
	}
#else
	ap_uint<10> angleRow = startRow - 15;
	ap_uint<10> angleCol = startCol - 15;
Loop_FillMergedRow:
	for(uint8_t _row = 0; _row < MAT_SIZE_ANGLE; _row++) {
Loop_FillMergedCol:
		for(uint8_t _col = 0; _col < MAT_SIZE_ANGLE; _col++) {
			#pragma HLS pipeline
			uint8_t v = cache[(angleRow+_row)%CACHE_LINES * MAX_W + (angleCol+_col)];
			//mAngle[_row * MAT_SIZE_ANGLE + _col] = v;
			mAngle.write(v);
			if(_row > 15 && _row <= MAT_SIZE_ANGLE-15 && _col > 15 && _col <= MAT_SIZE_ANGLE-15) {
				_dst.write((_row-16) * MAT_SIZE + (_col-16), v);
			}
		}
	}
#endif
}

void evaluateFast(xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1>& _src, hls::stream<kpt_t> &_dst, ap_uint<10> startRow, ap_uint<10> startCol) {
kpt_t kpt_out;
Loop_EvalRow:for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
	Loop_EvalCol:for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			#pragma HLS pipeline
			uint8_t r = _src.read(_mrow * MAT_SIZE + _mcol);
			kpt_out.global_x = _mcol + startCol;
			kpt_out.global_y = _mrow + startRow;
			kpt_out.angle_x = _mcol + 12; // -3 to offset FAST borders, +15 to translate to angle matrix dim
			kpt_out.angle_y = _mrow + 12;
			kpt_out.r = r;
			if(_mrow == MAT_SIZE-1 && _mcol == MAT_SIZE-1)
				kpt_out.done = 1;
			else
				kpt_out.done = 0;

			kpt_out.startRow = startRow;
			kpt_out.startCol = startCol;
			// Make sure we send the done flag
			//if((_mrow == MAT_SIZE-1 && _mcol == MAT_SIZE-1) || r > 0)
				_dst.write(kpt_out);
		}
	}
}

void undistort(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst) {
#if 1
Loop_UndistortRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_UndistortCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
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
		// NOTE_J: Datatypes?
		kpt.xU = uint2q<10,18>(kpt.global_x, 6) + undistortLookupX[kpt.startRow][kpt.startCol];
		kpt.yU = uint2q<10,18>(kpt.global_y, 6) + undistortLookupY[kpt.startRow][kpt.startCol];
		_dst.write(kpt);
	}
	while(kpt.done != 1);
#endif
}

void computeStereo(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, uint64_t* cacheDepth) {
#if 1
Loop_stereoRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_stereoCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt = _src.read();
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

void calcAngle(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, hls::stream<uint8_t>& _srcAngle) {
	const uint8_t u_max[16] = {15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3};
	#pragma HLS array_partition variable=u_max complete

	uint8_t mAngle[MAT_SIZE_ANGLE * MAT_SIZE_ANGLE];
	#pragma HLS array_partition variable=mAngle cyclic factor=62
Loop_fillAngleRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE_ANGLE; _mrow++) {
Loop_fillAngleCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE_ANGLE; _mcol++) {
			#pragma HLS pipeline
			mAngle[_mrow*MAT_SIZE_ANGLE + _mcol] = _srcAngle.read();
		}
	}

#if 1
Loop_angleRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_angleCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			kpt_t kpt = _src.read();
			ap_uint<10> row = kpt.angle_y;
			ap_uint<10> col = kpt.angle_x;
		
#if 0
			ap_int<24> m_01, m_10;
Loop_calcRow:for(ap_uint<5> _row = 0; _row <= 15; _row++) {
Loop_calcCol:	for(ap_int<6> _col = -15; _col <= 15; _col++) {
					if(_col >= -u_max[_row] && _col <= u_max[_row]) {
						if(_row == 0) {
							uint8_t val_minus = mAngle[(row - _row) * MAT_SIZE_ANGLE + (col + _col)];
							m_10 += _col * val_minus;
						}
						else {
							uint8_t val_minus = mAngle[(row - _row) * MAT_SIZE_ANGLE + (col + _col)];
							uint8_t val_plus = mAngle[(row + _row) * MAT_SIZE_ANGLE + (col + _col)];
							m_01 += _row * (val_plus - val_minus);
							m_10 += _col * (val_plus + val_minus);
						}
					}
				}
			}
#else
			ap_int<24> m_01, m_10, v_sum;
			for(ap_uint<5> _row = 0; _row <= 15; _row++) {
				v_sum = 0;
				for(ap_int<6> _col = -15; _col <= 15; _col++) {
					if(_col >= -u_max[_row] && _col <= u_max[_row]) {
						uint8_t val_plus = mAngle[(row + _row) * MAT_SIZE_ANGLE + (col + _col)];
						uint8_t val_minus = mAngle[(row - _row) * MAT_SIZE_ANGLE + (col + _col)];
						if(_row == 0) {
							v_sum += _col * val_minus;
							m_10 += _col * val_minus;
						}
						else {
							v_sum += (val_plus - val_minus);
							m_10 += _col * (val_plus + val_minus);
						}
					}
				}
				if(_row > 0)
					m_01 += _row * v_sum;
			}
#endif
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
	
		ap_int<24> m_01, m_10, v_sum;
		for(ap_uint<5> _row = 0; _row <= 15; _row++) {
			v_sum = 0;
			for(ap_int<6> _col = -15; _col <= 15; _col++) {
				if(_col >= -u_max[_row] && _col <= u_max[_row]) {
					uint8_t val_plus = mAngle[(row + _row) * MAT_SIZE_ANGLE + (col + _col)];
					uint8_t val_minus = mAngle[(row - _row) * MAT_SIZE_ANGLE + (col + _col)];
					v_sum += _row == 0 ? _col * val_minus : (val_plus - val_minus);
					m_10 += _row == 0 ? _col * val_minus : _col * (val_plus + val_minus);
				}
			}
			if(_row > 0)
				m_01 += _row * v_sum;
		}
		kpt.angle = (XF_PI_FIXED + xf::cv::Atan2LookupFP24(m_01, m_10, 24, 0, 24, 0));
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

#if 1
Loop_fillMemRow:
	for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_fillMemCol:
		for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
			kpt_t kpt = _src.read();
			if(kpt.r > 0) {
				// To not transfer negative numbers, add 512 constant to ap_int<18> candidates
				uint64_t dword0 = ((uint64_t)kpt.global_x << 48) | ((uint64_t)kpt.global_y << 32) | ((uint64_t)kpt.depth << 16) | kpt.r;
				uint64_t dword1 = ((uint64_t)(kpt.xU+512) << 32) | (kpt.yU+512);
				uint64_t dword2 = ((uint64_t)(kpt.xR+512) << 32) | kpt.angle;
				memOut[(read_index+1)*3 - 2] = dword0;
				memOut[(read_index+1)*3 - 1] = dword1;
				memOut[(read_index+1)*3 - 0] = dword2;
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
		uint64_t dword0 = ((uint64_t)kpt.global_x << 48) | ((uint64_t)kpt.global_y << 32) | ((uint64_t)kpt.depth << 16) | kpt.r;
		uint64_t dword1 = ((uint64_t)(kpt.xU+512) << 32) | (kpt.yU+512);
		uint64_t dword2 = ((uint64_t)(kpt.xR+512) << 32) | kpt.angle;
		memOut[(read_index+1)*3 - 2] = dword0;
		memOut[(read_index+1)*3 - 1] = dword1;
		memOut[(read_index+1)*3 - 0] = dword2;
		read_index++;
	} while(kpt.done != 1);
	memOut[0] = read_index;
#endif
}

//void dataflow_region(uint8_t* cache, BASETYPE* cacheDepth, uint8_t* mAngle, BASETYPE* memOut, ap_uint<10> startRow, ap_uint<10> startCol) {
void dataflow_region(uint8_t* cache, BASETYPE* cacheDepth, BASETYPE* memOut, ap_uint<10> startRow, ap_uint<10> startCol) {
	
	xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1> mFast_in(MAT_SIZE, MAT_SIZE);
	xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1> mFast_out(MAT_SIZE, MAT_SIZE);
	
	hls::stream<kpt_t> strm_eval2Undistort;
	hls::stream<kpt_t> strm_undistort2Stereo;
	hls::stream<kpt_t> strm_stereo2Angle;
	hls::stream<kpt_t> strm_angle2Mem;
	hls::stream<uint8_t> mAngle;
	#pragma HLS stream variable=strm_stereo2Angle depth=256
	//#pragma HLS stream variable=strm_angle2Mem depth=256

	{
		#pragma HLS dataflow
		populate_xfMat(cache, mAngle, mFast_in, startRow, startCol);
		xf::cv::fast<1, XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1>(mFast_in, mFast_out, FAST_TH);
		evaluateFast(mFast_out, strm_eval2Undistort, startRow, startCol);
		undistort(strm_eval2Undistort, strm_undistort2Stereo);
		computeStereo(strm_undistort2Stereo, strm_stereo2Angle, cacheDepth);
		calcAngle(strm_stereo2Angle, strm_angle2Mem, mAngle);
		fillMem(strm_angle2Mem, memOut);
	}
}

THREAD_ENTRY() {
	THREAD_INIT();

	BASETYPE ptr_i = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_d = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE _img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_h = MBOX_GET(rcsfast_sw2rt);

	ap_uint<6> NROWS = img_h == CC_H ? (img_h - 2*BORDER_EDGE) / WINDOW_SIZE : 1 + (img_h - 2*BORDER_EDGE) / WINDOW_SIZE;
	ap_uint<6> NCOLS = img_w == MAX_W ? (img_w - 2*BORDER_EDGE) / WINDOW_SIZE : 1 + (img_w - 2*BORDER_EDGE) / WINDOW_SIZE;

	uint8_t cache[MAX_W * CACHE_LINES];
	BASETYPE cacheDepth[MAX_W * CACHE_LINES];
	//uint8_t mAngle[MAT_SIZE_ANGLE * MAT_SIZE_ANGLE];
	//#pragma HLS array_partition variable=mAngle cyclic factor=62

	BASETYPE memOut[DWORDS_KPT*MAXPERBLOCK];

	read_next_batch(memif_hwt2mem, memif_mem2hwt, ptr_d, ptr_i, &cacheDepth[0], &cache[0], _img_w, img_h);
Loop_RowStep:
	for(ap_uint<6> rowStep = 0; rowStep < NROWS; rowStep++) {
		read_next_batch(memif_hwt2mem, memif_mem2hwt, ptr_d, ptr_i, &cacheDepth[0], &cache[0], _img_w, img_h);

		ap_uint<10> startRow = (BORDER_EDGE-3) + rowStep*WINDOW_SIZE;
		ap_uint<10> endRow = startRow + WINDOW_SIZE + 6;

Loop_ColStep:
		for(ap_uint<6> colStep = 0; colStep < NCOLS; colStep++) {
			ap_uint<10> startCol = (BORDER_EDGE-3) + colStep*WINDOW_SIZE;
			ap_uint<10> endCol = startCol + WINDOW_SIZE + 6;
	

			{ // Region 1
				//dataflow_region(&cache[0], &cacheDepth[0], &mAngle[0], &memOut[0], startRow, startCol);
				dataflow_region(&cache[0], &cacheDepth[0], &memOut[0], startRow, startCol);
			}

			BASETYPE _wroffset = MAXPERBLOCK * (rowStep*NCOLS + colStep);
			MEM_WRITE(&memOut[0], (ptr_o + (_wroffset*DWORDS_KPT*BYTES)), MAXPERBLOCK*DWORDS_KPT*BYTES);
		}
	}
	MBOX_PUT(rcsfast_rt2sw, DONEFLAG);
}
