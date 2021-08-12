extern "C" {
	#include "reconos_thread.h"
	#include "reconos_calls.h"
}
#include <iostream>
#include <cstring>

#if 1 // ReconOS64
    #define BASETYPE uint64_t
    #define BYTES 8
    #define MASK 7
	const BASETYPE _DONE = 0xffffffffffffffff;
#else // ReconOS32
    #define BASETYPE uint32_t
    #define BYTES 4
    #define MASK 3
	const BASETYPE _DONE = 0xffffffff;
#endif

#define MEM_READ1(src, dest, n) memcpy((void*)dest, (void*)src, n)
#define MEM_WRITE1(src, dest, n) memcpy((void*)dest, (void*)src, n)

#define CC_W 1280
#define CC_H 384
const BASETYPE BYTEMASK = 0xff;

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

#define SHIFT_NORM_SOBEL 3
const int8_t filterX[] = { 1, 2, 1,
						    0, 0, 0,
						   -1,-2,-1};

const int8_t filterY[] = { 1, 0, -1,
						   2, 0, -2,
						   1, 0, -1};

#define macro_prefetch_rows {\
    for(int i = 0; i < PREFETCH_ROWS; i++) {\
        _offset = ((ptr_i + i * img_w) & MASK);\
        _len = (img_w + _offset + BYTES)&(~MASK);\
        MEM_READ1(((ptr_i + i * img_w)&(~MASK)), &_in[0], _len);\
        for(int ii = 0; ii < CC_W; ii++) {\
            _byte_in_dword = (uint8_t)((_offset + ii) % BYTES);\
            _dword_ptr = (uint16_t)((_offset + ii) / BYTES);\
            _dword = _in[_dword_ptr];\
            _byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
            cache[i*CC_W + ii] = _byte;\
        }\
    }\
}

#define macro_read_row {\
    BASETYPE ptr_limit = (row-FILTER_SIZE_H) % img_h;\
    _offset = ((ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w) & MASK);\
    _len = (img_w + _offset + BYTES)&(~MASK);\
    MEM_READ1(((ptr_i + (PREFETCH_ROWS + (ptr_limit)) * img_w)&(~MASK)), &_in[0], _len);\
    for(int ii = 0; ii < CC_W; ii++) {\
        _byte_in_dword = (uint8_t)((_offset + ii) % BYTES);\
        _dword_ptr = (uint16_t)((_offset + ii) / BYTES);\
        _dword = _in[_dword_ptr];\
        _byte = ((_dword & (BYTEMASK << _byte_in_dword*8))>> _byte_in_dword*8);\
        _cache_line = CC_W * ((row+FILTER_SIZE_H+1) % CACHE_LINES);\
        cache[_cache_line + ii] = _byte;\
    }\
}

#define macro_write_row {\
	MEM_WRITE1(&_out[0], (ptr_o + row*CC_W), CC_W);\
}

THREAD_ENTRY() {
    // Variables needed for MEM_READ operations
    BASETYPE _offset, _dword, _cache_line, _len;
    uint16_t _dword_ptr;
    uint8_t _byte_in_dword, _byte;

	uint8_t cache[CC_W * CACHE_LINES];
	BASETYPE _in[CC_W/BYTES + 1];
	BASETYPE _out[CC_W/BYTES];

	BASETYPE ptr_i = MBOX_GET(rcs_sw2rt);
	BASETYPE img_w = MBOX_GET(rcs_sw2rt);
	BASETYPE img_h = MBOX_GET(rcs_sw2rt);
	BASETYPE ptr_o = MBOX_GET(rcs_sw2rt);
	BASETYPE mode = MBOX_GET(rcs_sw2rt);

	// Prefetch PREFETCH_ROWS lines of image
	macro_prefetch_rows;
	for(int row = FILTER_SIZE_H; row < img_h - FILTER_SIZE_H; row++) {
		macro_read_row;
		for(int i = 0; i < CC_W/BYTES; i++) {
			_out[i] = 0;
		}
	
		for(int col = FILTER_SIZE_H; col < img_w - FILTER_SIZE_H; col++) {
			// Reset temporary accumulation buffer
			uint16_t res = 0;
			int16_t resX = 0;
			int16_t resY = 0;

			uint16_t filter_ptr = 0;
			for(int i = -FILTER_SIZE_H; i <= FILTER_SIZE_H; i++) {
				for(int j = -FILTER_SIZE_H; j <= FILTER_SIZE_H; j++) {
					uint8_t _byte = cache[(row+i)%CACHE_LINES * CC_W + (col+j)];
					if(mode == 0) {
						res += _byte * filterU[filter_ptr];
					}
					else if(mode == 1) {
						res += _byte * filterG[filter_ptr];
					}
					else {
						resX += _byte * filterX[filter_ptr];
						resY += _byte * filterY[filter_ptr];
					}
					filter_ptr++;
				}
			}
			
			// Normalize result
			if(mode == 0) {
				_out[col/BYTES] |= ((BASETYPE)res) << 8*(col&MASK);
			}
			else if(mode == 1) {
				_out[col/BYTES] |= (((BASETYPE)(res >> SHIFT_NORM_GAUSS)) << 8*(col&MASK));
			}
			else {
				_out[col/BYTES] |= (((BASETYPE)(((uint16_t)abs(resX) + (uint16_t)abs(resY)) >> SHIFT_NORM_SOBEL)) << 8*(col&MASK));
			}
		}
		// Write-back computed row
		BASETYPE _addr = ptr_o + row*CC_W;
		//MBOX_PUT(rcs_rt2sw, _addr);
		macro_write_row;
	}

	MBOX_PUT(rcs_rt2sw, _DONE);
	THREAD_EXIT();
}
