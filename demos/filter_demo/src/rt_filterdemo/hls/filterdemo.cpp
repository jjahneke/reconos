#include "reconos_calls.h"
#include "reconos_thread.h"

#define CC_W 1280
#define CC_H 384
const uint64_t BYTEMASK = 0x00000000000000ff;

// Filter properties
#define FILTER_SIZE 3
#define FILTER_SIZE_H ((int)(FILTER_SIZE / 2))
#define CACHE_LINES (FILTER_SIZE + 1)
#define PREFETCH_ROWS (CACHE_LINES - 1)

// Unit kernel for verification
const uint8_t filterU[] = {0, 0, 0,
						   0, 1, 0,
						   0, 0, 0};

// Discrete approximation for Gaussian 3x3 kernel
// Divide by 16 -> shift right 4
#define SHIFT_NORM_GAUSS 4
const uint8_t filterG[] = {1,2,1,
						  2,4,2,
						  1,2,1};
//
//#define SHIFT_NORM_SOBEL 3
//const int8_t filterX[] = { 1, 2, 1,
//						    0, 0, 0,
//						   -1,-2,-1};
//
//const int8_t filterY[] = { 1, 0, -1,
//						   2, 0, -2,
//						   1, 0, -1};

#define macro_prefetch_rows {\
	for(int i = 0; i < PREFETCH_ROWS; i++) {\
		uint64_t _offset = (((uint64_t)ptr_i + i * img_w) & 7);\
		uint64_t _len = (img_w + _offset + 8)&(~7);\
		MEM_READ((((uint64_t)ptr_i + i * img_w)&(~7)), &_in[0], _len);\
		for(int ii = 0; ii < CC_W; ii++) {\
			uint8_t _byte_in_dword = (uint8_t)((_offset + ii) % 8);\
			uint16_t _dword_ptr = (uint16_t)((_offset + ii) / 8);\
			uint64_t _dword = _in[_dword_ptr];\
			uint8_t _byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
			cache[i*CC_W + ii] = _byte;\
		}\
	}\
}

#define macro_read_row {\
	uint64_t ptr_limit = (row-FILTER_SIZE_H) % img_h;\
	uint64_t _offset = (((uint64_t)ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w) & 7);\
	uint64_t _len = (img_w + _offset + 8)&(~7);\
	MEM_READ((((uint64_t)ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w)&(~7)), &_in[0], _len);\
	for(int ii = 0; ii < CC_W; ii++) {\
		uint64_t _offset = (((uint64_t)ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w) & 7);\
		uint8_t _byte_in_dword = (uint8_t)((_offset + ii) % 8);\
		uint16_t _dword_ptr = (uint16_t)((_offset + ii) / 8);\
		uint64_t _dword = _in[_dword_ptr];\
		uint8_t _byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
		uint64_t _cache_line = CC_W * ((row+FILTER_SIZE_H+1) % CACHE_LINES);\
		cache[_cache_line + ii] = _byte;\
	}\
}

#define macro_write_row {\
	MEM_WRITE(_out, (uint64_t)(ptr_o + row*CC_W), CC_W);\
}

THREAD_ENTRY() {
	THREAD_INIT();

	uint8_t cache[CC_W * CACHE_LINES];
	uint64_t _in[CC_W/8 + 1];
	uint64_t _out[CC_W/8];

	uint64_t ptr_i = MBOX_GET(rcs_sw2rt);
	uint64_t img_w = MBOX_GET(rcs_sw2rt);
	uint64_t img_h = MBOX_GET(rcs_sw2rt);
	uint64_t ptr_o = MBOX_GET(rcs_sw2rt);
	uint64_t mode = MBOX_GET(rcs_sw2rt);

	// Holds the result of filter operations on individual Byte
	uint16_t res;
	int16_t resX;
	int16_t resY;

	// Prefetch PREFETCH_ROWS lines of image
	macro_prefetch_rows;
	for(int row = FILTER_SIZE_H; row < CC_H - FILTER_SIZE_H; row++) {
		macro_read_row;
		for(int i = 0; i < CC_W/8; i++) {
			_out[i] = 0;
		}
	
		for(int col = FILTER_SIZE_H; col < CC_W - FILTER_SIZE_H; col++) {
			// Reset temporary accumulation buffer
			res = 0;
			resX = 0;
			resY = 0;

			uint16_t filter_ptr = 0;
			for(int i = -FILTER_SIZE_H; i <= FILTER_SIZE_H; i++) {
				for(int j = -FILTER_SIZE_H; j <= FILTER_SIZE_H; j++) {
					uint8_t _byte = cache[(row+i)%CACHE_LINES * CC_W + (col+j)];
					res += _byte * filterG[filter_ptr];
				
				//	if(mode == 0) {
				//		res += _byte * filterU[filter_ptr];
				//	}
				//	else if(mode == 1) {
				//		res += _byte * filterG[filter_ptr];
				//	}
				//	else {
				//		resX += _byte * filterX[filter_ptr];
				//		resY += _byte * filterY[filter_ptr];
				//	}

					filter_ptr++;
				}
			}
			
			// Normalize result
			//_out[col/8] |= (((uint64_t)(res >> SHIFT_NORM_GAUSS)) << 8*(col&7));
			_out[col/8] |= ((uint64_t)res) << 8*(col&7);

		//	if(mode == 0) {
		//		_out[col/8] |= ((uint64_t)res) << 8*(col&7);
		//	}
		//	else if(mode == 1) {
		//		_out[col/8] |= (((uint64_t)(res >> SHIFT_NORM_GAUSS)) << 8*(col&7));
		//	}
		//	else {
		//		_out[col/8] |= (((uint64_t)(((uint16_t)abs(resX) + (uint16_t)abs(resY)) >> SHIFT_NORM_SOBEL)) << 8*(col&7));
		//	}
		}
		// Write-back computed row
		macro_write_row;
	}

	MBOX_PUT(rcs_rt2sw, 0xffffffffffffffff);
}
