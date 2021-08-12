extern "C" {
	#include "reconos_thread.h"
	#include "reconos_calls.h"
}
#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#ifdef __aarch64__ // ReconOS64
	#define R64 1
    #define BASETYPE uint64_t
    #define BYTES 8
    #define MASK 7
	#define DWORDS_KPT 1
	#define DONEFLAG 0xffffffffffffffff
#else // ReconOS32
	#define R64 0
    #define BASETYPE uint32_t
    #define BYTES 4
    #define MASK 3
	#define DWORDS_KPT 2
	#define DONEFLAG 0xffffffff
#endif
	
#define MEM_READ1(src, dest, n) memcpy((void*)dest, (void*)src, n)
#define MEM_WRITE1(src, dest, n) memcpy((void*)dest, (void*)src, n)

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
#define MAXPERBLOCK 200

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((MAX_W - 2*BORDER_EDGE) / WINDOW_SIZE)

#define macro_read_next_batch {\
	for(int _row = 0; _row < WINDOW_SIZE; _row++) {\
		ptr_limit = row_count % img_h;\
		_offset = ((ptr_i + ptr_limit * _img_w) & MASK);\
		_len = (_img_w + _offset + BYTES)&(~MASK);\
		_addr = (ptr_i + ptr_limit * _img_w)&(~MASK);\
		MEM_READ1(_addr, &_in[0], _len);\
		for(int ii = 0; ii < MAX_W; ii++) {\
			uint8_t _b, _g, _r;\
			for(int b = 0; b < BYTEPERPIXEL; b++) {\
				_byte_in_dword = (uint8_t)((_offset + ii*BYTEPERPIXEL + b) % BYTES);\
				_dword_ptr = (uint16_t)((_offset + ii*BYTEPERPIXEL + b) / BYTES);\
				_dword = _in[_dword_ptr];\
				_byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
				if(b == 0)\
					_b = _byte;\
				else if(b == 1)\
					_g = _byte;\
				else if(b == 2)\
					_r = _byte;\
			}\
			_cache_line = MAX_W * (row_count % CACHE_LINES);\
			cache[_cache_line + ii] = kernel(_b, _g, _r);\
		}\
		row_count++;\
	}\
}

void populate_cvMat(uint8_t* cache, cv::Mat &mFast_in, uint16_t startRow, uint16_t startCol) {
	for(int _row = 0; _row < MAT_SIZE; _row++) {
		for(int _col = 0; _col < MAT_SIZE; _col++) {
			uint8_t v = cache[(startRow+_row)%CACHE_LINES * MAX_W + (startCol+_col)];
			mFast_in.at<uint8_t>(_row, _col) = v;
		}
	}
}

uint8_t kernel(uint8_t b, uint8_t r, uint8_t g) {
	return (uint8_t)(0.114*b + 0.587*r + 0.299*g);
}

THREAD_ENTRY() {
	// Variables needed for MEM_READ operations
	BASETYPE _offset, _dword, _cache_line, _len, _addr, ptr_limit;
	uint16_t _dword_ptr;
	uint8_t _byte_in_dword, _byte;

	uint8_t cache[MAX_W * CACHE_LINES];
	BASETYPE _in[CC_W/BYTES + 1];
	
	uint16_t row_count = 0;
	BASETYPE memOut[DWORDS_KPT];
	
	BASETYPE ptr_i = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_h = MBOX_GET(rcsfast_sw2rt);
	
	// Pixel length to Byte length
	unsigned int _img_w = img_w*BYTEPERPIXEL;

	// Prefetch BATCH lines of image
	macro_read_next_batch;
	for(int rowStep = 0; rowStep < NROWS; rowStep++) {
		macro_read_next_batch;

		uint16_t startRow = (BORDER_EDGE-3) + rowStep*WINDOW_SIZE;
		uint16_t endRow = startRow + WINDOW_SIZE + 6;

		for(int colStep = 0; colStep < NCOLS; colStep++) {
			uint16_t startCol = (BORDER_EDGE-3) + colStep*WINDOW_SIZE;
			uint16_t endCol = startCol + WINDOW_SIZE + 6;
	
			uint16_t local_cnt = 0;
			cv::Mat mFast_in = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_8UC1);
			std::vector<cv::KeyPoint> mFast_out;
			populate_cvMat(&cache[0], mFast_in, startRow, startCol);
			cv::FAST(mFast_in, mFast_out, FAST_TH, true);
			// Mat eval & memWrite;
			for(int _kpt = 0; _kpt < mFast_out.size(); _kpt++) {
				if(local_cnt >= MAXPERBLOCK)
					continue;
				cv::KeyPoint kp = mFast_out[_kpt];
				BASETYPE x = (BASETYPE)kp.pt.x + startCol;
				BASETYPE y = (BASETYPE)kp.pt.y + startRow;
				BASETYPE a = 0;
				BASETYPE r = (BASETYPE)kp.response;
				if(R64) {
					memOut[0] = (x << 48) | (y << 32) | (a << 16) | r;
				}
				else {
					memOut[0] = (x << 16) | y;
					memOut[1] = (a << 16) | r;
				}
				BASETYPE _offset = MAXPERBLOCK * (rowStep*NCOLS + colStep) + local_cnt;
				MEM_WRITE1(&memOut[0], (ptr_o + (_offset*DWORDS_KPT*BYTES)), DWORDS_KPT*BYTES);
				local_cnt++;
			}
		}
	}
	MBOX_PUT(rcsfast_rt2sw, DONEFLAG);
	THREAD_EXIT();
}
