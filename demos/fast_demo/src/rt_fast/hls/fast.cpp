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
#define FAST_TH 7
#define MAXPERBLOCK 100

const BASETYPE BYTEMASK = 0xff;
const BASETYPE SHORTMASK = 0xffff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAT_SIZE 48
#define MAT_SIZE_ANGLE 64
#define CACHE_LINES (WINDOW_SIZE * 2)

#if 1
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

/* Converts fixed to Q representation */
ap_uint<32> ufixed2q(ap_ufixed<20,12> v, uint8_t frac) {
	#pragma HLS inline
	return ((ap_fixed<32,24>)v << frac); // == v * (2 << (frac-1) == v * 2**frac
}

void read_next_batch(hls::stream<uint64_t>& memif_hwt2mem, hls::stream<uint64_t>& memif_mem2hwt, BASETYPE ptr_d, BASETYPE ptr_i, ap_uint<64>* cacheDepth, ap_uint<64>* cache, BASETYPE _img_w, BASETYPE img_h, uint16_t& row_count) {
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
Loop_batchReadCol:
		for(int ii = 0; ii < MAX_W/8; ii++) {
			#pragma HLS pipeline
			ap_uint<64> _v = 0;
			for(int b = 0; b < 8; b++) {
				uint16_t _dword_ptr = (_offset + ii*24 + BYTEPERPIXEL*b) / BYTES;
				uint8_t _bidw = (_offset + ii*24 + BYTEPERPIXEL*b) % BYTES;
				BASETYPE _dword0 = _in[_dword_ptr];
				BASETYPE _dword1 = _in[_dword_ptr+1];
				uint8_t _b = ((_dword0 & (BYTEMASK << _bidw*8)) >> _bidw*8);
				uint8_t _g = _bidw < 7 ? ((_dword0 & (BYTEMASK << (_bidw+1)*8)) >> (_bidw+1)*8) : _dword1 & BYTEMASK;
				uint8_t _r = _bidw < 6 ? ((_dword0 & (BYTEMASK << (_bidw+2)*8)) >> (_bidw+2)*8) : ((_dword1 & (BYTEMASK << (_bidw%6)*8)) >> (_bidw%6)*8);
				_v |= ((ap_uint<64>)kernel(_b, _g, _r) << 8*b); // Preserve little endian-ness
			}
			cache[MAX_W/8 * (row_count % CACHE_LINES) + ii] = _v;
		}
		row_count++;
	}
}

typedef struct {
	ap_uint<10> global_x, global_y;
	ap_uint<10> angle_x, angle_y;
	uint8_t r;
	int16_t angle; // NOTE_J: How to handle this one?
	ap_ufixed<20,12> xU, yU;
	ap_ufixed<20,12> xR;// Values in Q20.12.8
	uint16_t depth;
	ap_uint<1> done;
	ap_uint<64> desc[4];
} kpt_t;

void populate_xfMat(ap_uint<64>* cache, xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC>& _dstFast, hls::stream<ap_uint<64>>& _dstAngle, xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC>& _dstBlur, ap_uint<10> startRow, ap_uint<10> startCol) {
#if 1
Loop_FillRowFast:
	for(ap_uint<7> _row = 0; _row < MAT_SIZE; _row++) {
Loop_FillColFast:
		for(ap_uint<4> _col = 0; _col < MAT_SIZE/8; _col++) {
			#pragma HLS pipeline
			ap_uint<64> v = cache[((startRow - 8 + _row)%CACHE_LINES)*MAX_W/8 + (startCol/8 - 1 + _col)];
#if 0
			_dstFast.write(_row * MAT_SIZE/8 + _col, v); // Simple copy as endianness is preserved (xfExtractPixels requires little-endian dwords)
#else
			for(ap_uint<4> b = 0; b < 8; b++) {
				ap_uint<8> _v = (v & (BYTEMASK << 8*b)) >> 8*b; // To keep correct pixel order, have to access bytes in dword from LSB to MSB
				_dstFast.write(_row * MAT_SIZE + _col*8 + b, _v);
			}
#endif
		}
	}
Loop_FillRowBlur:
	for(ap_uint<7> _row = 0; _row < MAT_SIZE_ANGLE; _row++) {
Loop_FillColBlur:
		for(ap_uint<4> _col = 0; _col < MAT_SIZE_ANGLE/8; _col++) {
			#pragma HLS pipeline
			ap_uint<64> v = cache[((startRow - 16 + _row)%CACHE_LINES)*MAX_W/8 + (startCol/8 - 2 + _col)];
			_dstAngle.write(v); // Little-endian
#if 0 
			_dstBlur.write(_row * MAT_SIZE_ANGLE/8 + _col, v); // Simple copy as endianness is preserved (xfExtractPixels requires little-endian dwords)
#else
			for(ap_uint<4> b = 0; b < 8; b++) {
				ap_uint<8> _v = (v & (BYTEMASK << 8*b)) >> 8*b; //To keep correct pixel order, have to access bytes in dword from LSB to MSB
				_dstBlur.write(_row * MAT_SIZE_ANGLE + _col*8 + b, _v);
			}
#endif
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
				_dstFast.write((_row+1) * MAT_SIZE/8 + (_col+1), v);
			}
		}
	}
#endif
}

void filterFast(xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC>& _src, hls::stream<kpt_t>& _dst, ap_uint<10> startRow, ap_uint<10> startCol) {//, volatile int* counts) {
	ap_uint<6> cnt = 0;
Loop_EvalRow:
	for(ap_uint<6> _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
	Loop_EvalCol:
		for(ap_uint<6> _mcol = 0; _mcol < MAT_SIZE/TYPEDIV; _mcol++) {
			#pragma HLS pipeline
			kpt_t kpt;
			ap_uint<TYPEWIDTH> _r = _src.read(_mrow * MAT_SIZE/TYPEDIV + _mcol);

Loop_evalCalc:
			for(ap_uint<4> b = 0; b < TYPEDIV; b++) { // 8 Iterations for XF_NPPC8, 1 iteration for XF_NPPC1
				ap_uint<8> resp = (_r & (BYTEMASK << 8*b) >> 8*b);
				kpt.r = resp;
				kpt.global_x = startCol - 8 + _mcol*TYPEDIV + b;
				kpt.global_y = startRow - 8 + _mrow;
				kpt.angle_x = 8 + _mcol*TYPEDIV + b;
				kpt.angle_y = 8 + _mrow;
				// NOTE_J: Set flag when on pixel in row 39, col 39, i.e, on 31,31 in 32x32 matrix
				if((_mrow == ROI_ROW_HI && _mcol == ROI_COL_HI && b == TYPEDIV-1) || cnt == 32)
					kpt.done = 1;
				else
					kpt.done = 0;
	
				// NOTE_J: Only keep points inside 32x32 ROI and dummy keypoint
				bool inRoiLargerZero = (_mrow >= ROI_ROW_LO && _mrow <= ROI_ROW_HI && _mcol >= ROI_COL_LO && _mcol <= ROI_COL_HI && resp > 0);
				bool endOfRoi = (_mrow == ROI_ROW_HI && _mcol == ROI_COL_HI && b == TYPEDIV-1);
				if((inRoiLargerZero || endOfRoi) && cnt <= 32) {
//					*counts++;
					cnt++;
					_dst.write(kpt);
				}
			}
		}
	}
}

void undistort(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst) {//, volatile int* counts) {
	kpt_t kpt;
While_undistort:
	do {
		kpt = _src.read();
		// undistortLookup are stored as Q14.6.8 / ap_fixed<14,6>
		// ap_uint<10> + ap_fixed<14,6> => ap_ufixed<19, 11, 8>
		uint8_t idx = (kpt.global_x >> 4) - 1;
		uint8_t idy = (kpt.global_y >> 4) - 1;
		kpt.xU = kpt.global_x + undistortLookupX[idy][idx];
		kpt.yU = kpt.global_y + undistortLookupY[idy][idx];
		
//		*counts++;
		_dst.write(kpt);
	}
	while(kpt.done != 1);
}

void stereo(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, ap_uint<64>* cacheDepth, ap_uint<18> depthFactor) {//, volatile int* counts) {
	kpt_t kpt;
While_stereo:
	do {
		kpt = _src.read();
		
		ap_uint<10> x = kpt.global_x;
		ap_uint<10> y = kpt.global_y;
		uint16_t _addr = y*CC_W_DEPTH + (x/4); // x/4 to find dword
		ap_uint<2> _sidw = x % 4;
		ap_uint<16> d = (ap_uint<16>)(cacheDepth[_addr] & (SHORTMASK << _sidw*16) >> _sidw*16);

		kpt.depth = d; // Filter for kpt.d == 0
		if(d > 0)
			kpt.xR = kpt.xU - (ap_ufixed<20,12>)((ap_ufixed<26,18>)depthFactor/d);
		else
			kpt.xR = 0;

//		*counts++;
		_dst.write(kpt);
	}
	while(kpt.done != 1);
}

void calcAngle(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, hls::stream<ap_uint<64>>& _srcAngle) {//, volatile int* counts) {
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
			ap_uint<64> _v = _srcAngle.read();
			for(ap_uint<4> b = 0; b < 8; b++) {
				ap_uint<8> v = (_v & (BYTEMASK << 8*b) >> 8*b);
				mAngle[_mrow*MAT_SIZE_ANGLE + _mcol*8 + b] = v; // mAngle now contains pixels in correct order, i.e., [p0, p1, p2, ..., p7, p8, ...]
			}
		}
	}

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
//		*counts++;
		_dst.write(kpt);
	} while(kpt.done != 1);
}

void calcDescriptor(hls::stream<kpt_t>& _src, hls::stream<kpt_t>& _dst, xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC>& _srcBlur) {//, volatile int* counts) {
	ap_uint<8> mDesc[MAT_SIZE_ANGLE * MAT_SIZE_ANGLE];
	//#pragma HLS array_partition variable=mAngle cyclic factor=64
Loop_fillDescRow:
	for(ap_uint<7> _mrow = 0; _mrow < MAT_SIZE_ANGLE; _mrow++) {
Loop_fillDescCol:
		for(ap_uint<7> _mcol = 0; _mcol < MAT_SIZE_ANGLE/TYPEDIV; _mcol++) {
			#pragma HLS pipeline
			ap_uint<TYPEWIDTH> _v = _srcBlur.read(_mrow * MAT_SIZE_ANGLE/TYPEDIV + _mcol);
Loop_unpackPixels:
			for(ap_uint<4> b = 0; b < TYPEDIV; b++) {
				ap_uint<8> v = (_v & (BYTEMASK << 8*b) >> 8*b);
				mDesc[_mrow*MAT_SIZE_ANGLE + _mcol*TYPEDIV + b] = v;
			}
		}
	}

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
				kpt.desc[dw].range(bit,bit) = mDesc[(y+pair.p1y)*MAT_SIZE_ANGLE + (x+pair.p1x)] < mDesc[(y+pair.p2y)*MAT_SIZE_ANGLE + (x+pair.p2x)] ? 1 : 0;
			}
		}
//		*counts++;
		_dst.write(kpt);
	} while(kpt.done != 1);
}

/**
[kp.x, kp.y ; depth, r] [kpU.X ; kpU.y] [kpR ; a] // If a is converted float on FPGA
[kp.x, kp.y ; depth, r] [kpU.X ; kpU.y] [kpR ; 0, a] // If a is converted to float on CPU
[kp.x, kp.y ; a, r] [kpU.X, kpU.y ; kpR, depth] // If all ap_fixed are converted to float on CPU
*/
void fillMem(hls::stream<kpt_t>& _src, BASETYPE* memOut) {//, volatile int* counts) {
	ap_uint<10> read_index = 0;

	kpt_t kpt;
While_fillMem:
	do {
		kpt = _src.read();
		memOut[(read_index+1)*7 - 6] = ((uint64_t)kpt.global_x << 48) | ((uint64_t)kpt.global_y << 32) | ((uint64_t)kpt.depth << 16) | kpt.r;
		memOut[(read_index+1)*7 - 5] = (uint64_t)((ufixed2q(kpt.xU, 8)) << 32) | ufixed2q(kpt.yU, 8);
		memOut[(read_index+1)*7 - 4] = (uint64_t)((ufixed2q(kpt.xR, 8)) << 32) | kpt.angle;
		memOut[(read_index+1)*7 - 3] = kpt.desc[3];
		memOut[(read_index+1)*7 - 2] = kpt.desc[2];
		memOut[(read_index+1)*7 - 1] = kpt.desc[1];
		memOut[(read_index+1)*7 - 0] = kpt.desc[0];
		read_index++;
//		*counts++;
	} while(kpt.done != 1);
	memOut[0] = read_index;
}

//void dataflow_region(ap_uint<64>* cache, ap_uint<64>* cacheDepth, BASETYPE* memOut, ap_uint<10> startRow, ap_uint<10> startCol, ap_uint<18> depthFactor, volatile int* count0, volatile int* count1, volatile int* count2, volatile int* count3, volatile int* count4, volatile int* count5) {
void dataflow_region(ap_uint<64>* cache, ap_uint<64>* cacheDepth, BASETYPE* memOut, ap_uint<10> startRow, ap_uint<10> startCol, ap_uint<18> depthFactor) {
	
	xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC> mFast_in(MAT_SIZE, MAT_SIZE);
	xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, NPPC> mFast_out(MAT_SIZE, MAT_SIZE);
	
	xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC> mBlur_in(MAT_SIZE_ANGLE, MAT_SIZE_ANGLE);
	xf::cv::Mat<XF_8UC1, MAT_SIZE_ANGLE, MAT_SIZE_ANGLE, NPPC> mBlur_out(MAT_SIZE_ANGLE, MAT_SIZE_ANGLE);
	
	hls::stream<kpt_t> strm_eval2Undistort;
	hls::stream<kpt_t> strm_undistort2Stereo;
	hls::stream<kpt_t> strm_stereo2Angle;
	hls::stream<kpt_t> strm_angle2Desc;
	hls::stream<kpt_t> strm_desc2Mem;
	hls::stream<ap_uint<64>> mAngle;
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
		filterFast(mFast_out, strm_eval2Undistort, startRow, startCol);//, count0);
		undistort(strm_eval2Undistort, strm_undistort2Stereo);//, count1);
		stereo(strm_undistort2Stereo, strm_stereo2Angle, cacheDepth, depthFactor);//, count2);
		calcAngle(strm_stereo2Angle, strm_angle2Desc, mAngle);//, count3);
		calcDescriptor(strm_angle2Desc, strm_desc2Mem, mBlur_out);//, count4);
		fillMem(strm_desc2Mem, memOut);//, count5);
	}
}

THREAD_ENTRY() {
	THREAD_INIT();

/* These are pragmas to be applied to variables in the header file */
//	#pragma HLS data_pack variable=pattern[0]
//	#pragma HLS data_pack variable=pattern[1]

	ap_uint<64> cache[MAX_W/8 * CACHE_LINES];
	//#pragma HLS array_partition variable=cache cyclic factor=640
	uint16_t row_count = 0;
	ap_uint<64> cacheDepth[CC_W_DEPTH * CACHE_LINES];
	BASETYPE memOut[DWORDS_KPT*MAXPERBLOCK];
	#pragma HLS array_partition variable=memOut cyclic factor=7

	BASETYPE ptr_i = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_d = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE _img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_h = MBOX_GET(rcsfast_sw2rt);
	ap_uint<18> depthFactor = (ap_uint<18>)MBOX_GET(rcsfast_sw2rt);

	ap_uint<6> NROWS = img_h == CC_H ? (img_h - 2*BORDER_EDGE) / WINDOW_SIZE : 1 + (img_h - 2*BORDER_EDGE) / WINDOW_SIZE;
	ap_uint<6> NCOLS = img_w == MAX_W ? (img_w - 2*BORDER_EDGE) / WINDOW_SIZE : 1 + (img_w - 2*BORDER_EDGE) / WINDOW_SIZE;

	read_next_batch(memif_hwt2mem, memif_mem2hwt, ptr_d, ptr_i, &cacheDepth[0], &cache[0], _img_w, img_h, row_count);
Loop_RowStep:
	for(ap_uint<6> rowStep = 0; rowStep < NROWS; rowStep++) {
		read_next_batch(memif_hwt2mem, memif_mem2hwt, ptr_d, ptr_i, &cacheDepth[0], &cache[0], _img_w, img_h, row_count);

		ap_uint<10> startRow = BORDER_EDGE + rowStep*WINDOW_SIZE;
		ap_uint<10> endRow = startRow + WINDOW_SIZE + 6;

Loop_ColStep:
		for(ap_uint<6> colStep = 0; colStep < NCOLS; colStep++) {
			ap_uint<10> startCol = BORDER_EDGE + colStep*WINDOW_SIZE;
			ap_uint<10> endCol = startCol + WINDOW_SIZE + 6;
//			volatile int count0 = -1;
//			volatile int count1 = -1;
//			volatile int count2 = -1;
//			volatile int count3 = -1;
//			volatile int count4 = -1;
//			volatile int count5 = -1;

			{ // Region 1
				dataflow_region(&cache[0], &cacheDepth[0], &memOut[0], startRow, startCol, depthFactor);//, &count0, &count1, &count2, &count3, &count4, &count5);
			}

			BASETYPE _wroffset = MAXPERBLOCK * (rowStep*NCOLS + colStep);
			MEM_WRITE(&memOut[0], (ptr_o + (_wroffset*DWORDS_KPT*BYTES)), MAXPERBLOCK*DWORDS_KPT*BYTES);
//			MBOX_PUT(rcsfast_rt2sw, count0);
//			MBOX_PUT(rcsfast_rt2sw, count1);
//			MBOX_PUT(rcsfast_rt2sw, count2);
//			MBOX_PUT(rcsfast_rt2sw, count3);
//			MBOX_PUT(rcsfast_rt2sw, count4);
//			MBOX_PUT(rcsfast_rt2sw, count5);
		}
	}
	MBOX_PUT(rcsfast_rt2sw, DONEFLAG);
}
