#include "reconos_thread.h"
#include "reconos_calls.h"

#include "common/xf_common.hpp"
#include "features/xf_fast.hpp"

#define CC_W 1280
#define CC_H 384
const uint64_t BYTEMASK = 0x00000000000000ff;

#define FAST_TH 7
#define MAXPERBLOCK 200
#define DWORDS_KPT 2

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAT_SIZE (WINDOW_SIZE + 6)
#define CACHE_LINES (WINDOW_SIZE * 2)
#define PREFETCH_ROWS (CACHE_LINES)

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((CC_W - 2*BORDER_EDGE) / WINDOW_SIZE)

#define macro_prefetch_rows {\
	for(int i = 0; i < PREFETCH_ROWS; i++) {\
		uint64_t _offset = (((uint64_t)ptr_i + i * img_w) & 7);\
		MEM_READ((((uint64_t)ptr_i + i * img_w)&(~7)), &_in[0], CC_W + 8);\
		for(int ii = 0; ii < CC_W; ii++) {\
			uint8_t _byte_in_dword = (uint8_t)((_offset + ii) % 8);\
			uint16_t _dword_ptr = (uint16_t)((_offset + ii) / 8);\
			uint64_t _dword = _in[_dword_ptr];\
			uint8_t _byte = ((_dword & (BYTEMASK << _byte_in_dword*8)) >> _byte_in_dword*8);\
			cache[i*CC_W + ii] = _byte;\
		}\
		row_count++;\
	}\
}

#define macro_read_next_batch {\
	for(int _row = 0; _row < WINDOW_SIZE; _row++) {\
		uint64_t ptr_limit = row_count % img_h;\
		MEM_READ((((uint64_t)ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w)&(~7)), &_in[0], CC_W + 8);\
		for(int ii = 0; ii < CC_W; ii++) {\
			uint64_t _offset = (((uint64_t)ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w) & 7);\
			uint8_t _byte_in_dword = (uint8_t)((_offset + ii) % 8);\
			uint16_t _dword_ptr = (uint16_t)((_offset + ii) / 8);\
			uint64_t _dword = _in[_dword_ptr];\
			uint8_t _byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
			uint64_t _cache_line = CC_W * (row_count % CACHE_LINES);\
			cache[_cache_line + ii] = _byte;\
		}\
		row_count++;\
	}\
}

#define populate_xfMat {\
	for(int _row = 0; _row < MAT_SIZE; _row++) {\
		for(int _col = 0; _col < MAT_SIZE; _col++) {\
			uint8_t v = cache[(startRow+_row)%CACHE_LINES * CC_W + (startCol+_col)];\
			mFast_in.write(_row * MAT_SIZE + _col, v);\
		}\
	}\
}

THREAD_ENTRY() {
	THREAD_INIT();
	uint8_t cache[CC_W * CACHE_LINES];
	uint64_t _in[CC_W/8 + 1];
	uint16_t row_count = 0;
	uint64_t memOut[DWORDS_KPT];

	uint64_t ptr_i = MBOX_GET(rcsfast_sw2rt);
	uint64_t img_w = MBOX_GET(rcsfast_sw2rt);
	uint64_t img_h = MBOX_GET(rcsfast_sw2rt);
	uint64_t ptr_o = MBOX_GET(rcsfast_sw2rt);

	// Prefetch PREFETCH_ROWS lines of image
	macro_prefetch_rows;
	for(int rowStep = 0; rowStep < NROWS; rowStep++) {
		if(rowStep > 0)
			macro_read_next_batch;

		uint16_t startRow = (BORDER_EDGE-3) + rowStep*WINDOW_SIZE;
		uint16_t endRow = startRow + WINDOW_SIZE + 6;

		for(int colStep = 0; colStep < NCOLS; colStep++) {
			uint16_t startCol = (BORDER_EDGE-3) + colStep*WINDOW_SIZE;
			uint16_t endCol = startCol + WINDOW_SIZE + 6;
	
			uint16_t local_cnt = 0;
			xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1> mFast_in(MAT_SIZE, MAT_SIZE);
			xf::cv::Mat<XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1> mFast_out(MAT_SIZE, MAT_SIZE);

			{
			#pragma HLS dataflow
			populate_xfMat;
			xf::cv::fast<1, XF_8UC1, MAT_SIZE, MAT_SIZE, XF_NPPC1>(mFast_in, mFast_out, FAST_TH);
			// Mat eval & memWrite;
			for(uint8_t _mrow = 0; _mrow < MAT_SIZE; _mrow++) {
				for(uint8_t _mcol = 0; _mcol < MAT_SIZE; _mcol++) {
					uint64_t r = mFast_out.read(_mrow * MAT_SIZE + _mcol);
					uint64_t x = _mcol + startCol;
					uint64_t y = _mrow + startRow;
					uint64_t a = 0;
					if(r > 0) {
						uint64_t memOut[DWORDS_KPT];
						memOut[0] = (x << 32) | y;
						memOut[1] = (a << 32) | r;
						uint64_t _offset = MAXPERBLOCK * (rowStep*NCOLS + colStep) + local_cnt;
						MEM_WRITE(memOut, (ptr_o + (_offset*DWORDS_KPT*8)), DWORDS_KPT*8);
						local_cnt++;
					}
				}
			}
			} // End dataflow
		}
	}

	MBOX_PUT(rcsfast_rt2sw, 0xffffffffffffffff);
}
