#include "reconos_thread.h"
#include "reconos_calls.h"

#include "common/xf_common.hpp"
#include "features/xf_fast.hpp"

#define BASETYPE uint64_t
#define BYTES 8
#define MASK 7
#define DWORDS_KPT 1
#define DONEFLAG 0xffffffffffffffff
	
#define MAX_W 640 // In pixel
#define BYTEPERPIXEL 3
#define CC_W (MAX_W*BYTEPERPIXEL) // In Byte
#define CC_H 480
const BASETYPE BYTEMASK = 0xff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAT_SIZE (WINDOW_SIZE + 6)
#define CACHE_LINES (WINDOW_SIZE * 2)

#define FAST_TH 7
#define MAXPERBLOCK 100

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((MAX_W - 2*BORDER_EDGE) / WINDOW_SIZE)

#define macro_read_next_batch {\
	for(int _row = 0; _row < WINDOW_SIZE; _row++) {\
		BASETYPE ptr_limit = row_count % img_h;\
		BASETYPE _offset = ((ptr_i + ptr_limit * _img_w) & MASK);\
		BASETYPE _len = (_img_w + _offset + BYTES)&(~MASK);\
		BASETYPE _addr = (ptr_i + ptr_limit * _img_w)&(~MASK);\
		MEM_READ(_addr, &_in[0], _len);\
		for(int ii = 0; ii < MAX_W; ii++) {\
			uint16_t _dword_ptr = (uint16_t)((_offset + ii*BYTEPERPIXEL) / BYTES);\
			uint8_t _byte_in_dword = (uint8_t)((_offset + ii*BYTEPERPIXEL) % BYTES);\
			BASETYPE _dword0 = _in[_dword_ptr];\
			BASETYPE _dword1 = _in[_dword_ptr+1];\
			uint8_t _b = ((_dword0 & (BYTEMASK << _byte_in_dword*8)) >> _byte_in_dword*8);\
			uint8_t _g = _byte_in_dword < 7 ? ((_dword0 & (BYTEMASK << (_byte_in_dword+1)*8)) >> (_byte_in_dword+1)*8) : _dword1 & BYTEMASK;\
			uint8_t _r = _byte_in_dword < 6 ? ((_dword0 & (BYTEMASK << (_byte_in_dword+2)*8)) >> (_byte_in_dword+2)*8) : ((_dword1 & (BYTEMASK << (_byte_in_dword%6)*8)) >> (_byte_in_dword%6)*8);\
			BASETYPE _cache_line = MAX_W * (row_count % CACHE_LINES);\
			cache[_cache_line + ii] = kernel(_b, _g, _r);\
		}\
		row_count++;\
	}\
}

void populate_xfMat(uint8_t cache[MAX_W*CACHE_LINES], xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1>& _dst, uint16_t startRow, uint16_t startCol) {
Loop_FillRow:
	for(uint8_t _row = 0; _row < MAT_SIZE; _row++) {
Loop_FillCol:
		for(uint8_t _col = 0; _col < MAT_SIZE; _col++) {
			uint8_t v = cache[(startRow+_row)%CACHE_LINES * MAX_W + (startCol+_col)];
			_dst.write(_row * MAT_SIZE + _col, v);
		}
	}
}

#define populate_xfMat {\
	for(int _row = 0; _row < MAT_SIZE; _row++) {\
		for(int _col = 0; _col < MAT_SIZE; _col++) {\
			uint8_t v = cache[(startRow+_row)%CACHE_LINES * MAX_W + (startCol+_col)];\
			mFast_in.write(_row * MAT_SIZE + _col, v);\
		}\
	}\
}

uint8_t kernel(uint8_t b, uint8_t g, uint8_t r) {
	#pragma HLS inline
	return (uint8_t)(0.114*b + 0.587*g + 0.299*r);
}

THREAD_ENTRY() {
	THREAD_INIT();

	BASETYPE ptr_i = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE _img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_h = MBOX_GET(rcsfast_sw2rt);
	
	uint8_t cache[MAX_W * CACHE_LINES];
	BASETYPE _in[CC_W/BYTES + 1];
	uint16_t row_count = 0;

	// Prefetch BATCH lines of image
	macro_read_next_batch;
Loop_RowStep:
	for(uint8_t rowStep = 0; rowStep < NROWS; rowStep++) {
		macro_read_next_batch;

		uint16_t startRow = (BORDER_EDGE-3) + rowStep*WINDOW_SIZE;
		uint16_t endRow = startRow + WINDOW_SIZE + 6;

Loop_ColStep:
		for(uint8_t colStep = 0; colStep < NCOLS; colStep++) {
			uint16_t startCol = (BORDER_EDGE-3) + colStep*WINDOW_SIZE;
			uint16_t endCol = startCol + WINDOW_SIZE + 6;
	
			uint16_t local_cnt = 0;
			BASETYPE memOut[DWORDS_KPT*MAXPERBLOCK];
			xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1> mFast_in(MAT_SIZE, MAT_SIZE);
			xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1> mFast_out(MAT_SIZE, MAT_SIZE);

			{
			#pragma HLS dataflow
			populate_xfMat;
			xf::cv::fast<1, XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1>(mFast_in, mFast_out, FAST_TH);
			// Mat eval & memWrite;
Loop_EvalRow:
			for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
Loop_EvalCol:
				for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
					BASETYPE r = mFast_out.read(_mrow * MAT_SIZE + _mcol);
					if(r > 0) {
						BASETYPE x = _mcol + startCol;
						BASETYPE y = _mrow + startRow;
						BASETYPE a = 0;
						memOut[local_cnt+1] = (x << 48) | (y << 32) | (a << 16) | r;
						local_cnt++;
					}
				}
				memOut[0] = local_cnt;
			}
			} // end dataflow
			BASETYPE _wroffset = MAXPERBLOCK * (rowStep*NCOLS + colStep);
			MEM_WRITE(&memOut[0], (ptr_o + (_wroffset*DWORDS_KPT*BYTES)), MAXPERBLOCK*DWORDS_KPT*BYTES);
		}
	}
	MBOX_PUT(rcsfast_rt2sw, DONEFLAG);
}
