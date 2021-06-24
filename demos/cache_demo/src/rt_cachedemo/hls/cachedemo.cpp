#include "reconos_calls.h"
#include "reconos_thread.h"

#include "common/xf_common.hpp"
#include "features/xf_fast.hpp"

#define CC_W 1280
#define CC_H 384

#define WS 60
#define FAST_WS 30
#define DWORDSPERLINE 7

#define macro_read_batch {for(int i=0; i<WS; i++) {\
			if(cache_cnt >= CC_H) {break;}\
			uint64_t _ptr_limit = cache_cnt % img_h;\
			uint64_t _offset = ((ptr + cache_cnt * img_w) & 7);\
			uint64_t _len = ((int)(img_w/8)+1 + _offset + 7)&(~7);\
			MEM_READ((uint64_t)((ptr + _ptr_limit * img_w)&(~7)), &cache[(CC_W/8) * cache_cnt], _len);\
			cache_cnt+=1;} }

#define macro_read_64 {\
			uint64_t _ptr_limit = 0;\
			uint64_t _offset = ((ptr + 0 * img_w) & 7);\
			uint64_t _len = (8 + _offset + 7)&(~7);\
			MEM_READ((uint64_t)((ptr + _ptr_limit * img_w)&(~7)), &buf8[0], _len);\
			}

#define macro_read_128 {\
			uint64_t _ptr_limit = 0;\
			uint64_t _offset = ((ptr + 0 * img_w) & 7);\
			uint64_t _len = (16 + _offset + 7)&(~7);\
			MEM_READ((uint64_t)((ptr + _ptr_limit * img_w)&(~7)), &buf16[0], _len);\
			}

#define macro_fill_mat {\
	uint8_t* _src = (uint8_t*)&cache[0];\
	uint8_t* dst = &mat1[0];\
    for(int i=0; i<FAST_WS; i++){\
        uint64_t row_index = iniY + i;\
        uint64_t byte_offset = ((uint64_t)ptr + row_index * img_w) & 7;\
        for(int j=0; j<FAST_WS; j++){\
            uint64_t col_index = iniX + j;\
            uint8_t v = _src[(row_index * CC_W + byte_offset + col_index)];\
            dst[i*FAST_WS+j] = v;\
		}\
    }\
}

void fill_remainder(uint64_t mem[7]) {
	uint32_t __mem[8] = {10,20,30,40,50,60,70,80};
	for(int i = 0; i < 4; i++){
		mem[i+3] = (((uint64_t)__mem[2*i]) << 32) | ((uint64_t)__mem[2*i+1]);
	}
}

THREAD_ENTRY() {
	THREAD_INIT();
	
	uint64_t cache_cnt = 0;
	uint64_t cache[CC_W/8 * CC_H];
	uint8_t mat1[FAST_WS*FAST_WS];
	xf::cv::Mat<XF_8UC1, FAST_WS, FAST_WS, XF_NPPC1> xfCvMat(FAST_WS, FAST_WS);
	uint32_t _mem[5] = {1,2,3,4,5};
	uint64_t mem[7];
	for(int i = 0; i < 2; i++){
		mem[i] = (((uint64_t)_mem[2*i]) << 32) | ((uint64_t)_mem[2*i+1]);
	}
	mem[2] = (((uint64_t)_mem[4]) << 32);
	fill_remainder(mem);

	uint64_t ptr =   MBOX_GET(resources_sw2rt);
	uint64_t img_w = MBOX_GET(resources_sw2rt);
	uint64_t img_h = MBOX_GET(resources_sw2rt);
	uint64_t ret_ptr = MBOX_GET(resources_sw2rt);
	uint64_t ret_ptr2 = MBOX_GET(resources_sw2rt);

	for(int i = 0; i < 7; i++){
		macro_read_batch;
		MBOX_PUT(resources_rt2sw, cache_cnt-WS);
		
		for(int ii = 0; ii < 8; ii++){
			MBOX_PUT(resources_rt2sw, (uint64_t)(cache[(CC_W/8) * (cache_cnt - WS) + ii]));
		}
	}

	for(int i = 0; i < 9; i++){
		MBOX_PUT(resources_rt2sw, (uint64_t)(cache[(CC_W/8) * i + 0]));
		MBOX_PUT(resources_rt2sw, (uint64_t)(cache[(CC_W/8) * i + 1]));
	}

	uint64_t iniX = 25;
	uint64_t iniY = 25;
	macro_fill_mat;

	for(int i = 0; i < FAST_WS; i++){
		for(int ii = 0; ii < FAST_WS; ii++){
			MBOX_PUT(resources_rt2sw, mat1[i*FAST_WS+ii]);
		}
	}

	for(int i = 0; i < FAST_WS; i++){
		for(int ii = 0; ii < FAST_WS; ii++){
			xfCvMat.write(i*FAST_WS+ii, mat1[i*FAST_WS+ii]);
			MBOX_PUT(resources_rt2sw, xfCvMat.read(i*FAST_WS+ii));
		}
	}

	for(int i = 0; i < 4; i++){
		MBOX_PUT(resources_rt2sw, (ret_ptr + i*8*DWORDSPERLINE));
		MEM_WRITE(mem, (ret_ptr + i*8*DWORDSPERLINE), DWORDSPERLINE*8);
	}
	
	// Second mem Test
	uint8_t _memTest[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
	MEM_WRITE(&_memTest[0], ret_ptr2, 16);

	// Done
	MBOX_PUT(resources_rt2sw, 0xffffffffffffffff);
}
