extern "C" {
	#include "reconos_thread.h"
	#include "reconos_calls.h"
}
#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define BASETYPE uint64_t
#define BYTES 8
#define MASK 7
#define DWORDS_KPT 1
#define DONEFLAG 0xffffffffffffffff
	
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
#define MAXPERBLOCK 100

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((MAX_W - 2*BORDER_EDGE) / WINDOW_SIZE)

#define macro_read_next_batch {\
	for(int _row = 0; _row < WINDOW_SIZE; _row++) {\
		BASETYPE ptr_limit = row_count % img_h;\
		BASETYPE _offset = ((ptr_i + ptr_limit * _img_w) & MASK);\
		BASETYPE _len = (_img_w + _offset + BYTES)&(~MASK);\
		BASETYPE _addr = (ptr_i + ptr_limit * _img_w)&(~MASK);\
		MEM_READ1(_addr, &_in[0], _len);\
		for(int ii = 0; ii < MAX_W; ii++) {\
			uint8_t _b, _g, _r;\
			for(int b = 0; b < BYTEPERPIXEL; b++) {\
				uint8_t _byte_in_dword = (uint8_t)((_offset + ii*BYTEPERPIXEL + b) % BYTES);\
				uint16_t _dword_ptr = (uint16_t)((_offset + ii*BYTEPERPIXEL + b) / BYTES);\
				BASETYPE _dword = _in[_dword_ptr];\
				uint8_t _byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
				if(b == 0)\
					_b = _byte;\
				else if(b == 1)\
					_g = _byte;\
				else if(b == 2)\
					_r = _byte;\
			}\
			BASETYPE _cache_line = MAX_W * (row_count % CACHE_LINES);\
			uint8_t _v = kernel(_b, _g, _r);\
			hash1 ^= _v;\
			cache[_cache_line + ii] = _v;\
		}\
		row_count++;\
	}\
}

uint8_t populate_cvMat(uint8_t* cache, cv::Mat &mFast_in, uint16_t startRow, uint16_t startCol, uint8_t hash) { 
Loop_FillRow:
	for(uint8_t _row = 0; _row < MAT_SIZE; _row++) {
Loop_FillCol:
		for(uint8_t _col = 0; _col < MAT_SIZE; _col++) {
			uint8_t v = cache[(startRow+_row)%CACHE_LINES * MAX_W + (startCol+_col)];
			hash ^= v;
			mFast_in.at<uint8_t>(_row, _col) = v;
		}
	}
	return hash;
}

uint8_t kernel(uint8_t b, uint8_t r, uint8_t g) {
	return (uint8_t)(0.114*b + 0.587*r + 0.299*g);
}

THREAD_ENTRY() {
	
	BASETYPE ptr_i = MBOX_GET(rcsfast_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_w = MBOX_GET(rcsfast_sw2rt);
	BASETYPE img_h = MBOX_GET(rcsfast_sw2rt);
	
	// Pixel length to Byte length
	unsigned int _img_w = img_w*BYTEPERPIXEL;

	uint8_t cache[MAX_W * CACHE_LINES];
	BASETYPE _in[CC_W/BYTES + 1];
	uint8_t hash1 = 0;
	uint8_t hash2 = 0;
	uint16_t row_count = 0;

	// Prefetch BATCH lines of image
	macro_read_next_batch;
Loop_RowStep:
	for(uint8_t rowStep = 0; rowStep < NROWS; rowStep++) {
		macro_read_next_batch;
		MBOX_PUT(rcsfast_rt2sw, (uint64_t)hash1 | ((uint64_t)rowStep << 16) | (0xffff000000000000));

		uint16_t startRow = (BORDER_EDGE-3) + rowStep*WINDOW_SIZE;
		uint16_t endRow = startRow + WINDOW_SIZE + 6;

Loop_ColStep:
		for(uint8_t colStep = 0; colStep < NCOLS; colStep++) {
			uint16_t startCol = (BORDER_EDGE-3) + colStep*WINDOW_SIZE;
			uint16_t endCol = startCol + WINDOW_SIZE + 6;
	
			uint16_t local_cnt = 0;
			BASETYPE memOut[DWORDS_KPT*MAXPERBLOCK];
			cv::Mat mFast_in = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_8UC1);
			std::vector<cv::KeyPoint> mFast_out;
			hash2 = populate_cvMat(&cache[0], mFast_in, startRow, startCol, hash2);
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
				memOut[local_cnt+1] = (x << 48) | (y << 32) | (a << 16) | r;
				local_cnt++;
			}
			memOut[0] = local_cnt;
			BASETYPE _wroffset = MAXPERBLOCK * (rowStep*NCOLS + colStep);
			MBOX_PUT(rcsfast_rt2sw, (uint64_t)hash2 | ((uint64_t)rowStep << 16) | ((uint64_t)colStep << 32));
			MEM_WRITE1(&memOut[0], (ptr_o + (_wroffset*DWORDS_KPT*BYTES)), MAXPERBLOCK*DWORDS_KPT*BYTES);

		}
	}
	MBOX_PUT(rcsfast_rt2sw, DONEFLAG);
	THREAD_EXIT();
}
