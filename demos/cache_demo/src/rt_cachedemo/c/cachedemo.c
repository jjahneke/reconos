#include "reconos_thread.h"
#include "reconos_calls.h"
#include "string.h"
#include "inttypes.h"
#include "stdio.h"

#define CC_W 1280
#define CC_H 384

#define WS 60

#define MEM_READ1(src, dest, n) memcpy((void*)dest, (void*)src, n)

#define macro_read_batch {for(int i=0; i<WS; i++) {\
			if(cache_cnt >= CC_H) {break;}\
			uint64_t _ptr_limit = cache_cnt % img_h;\
			uint64_t _offset = ((ptr + cache_cnt * img_w) & 7);\
			uint64_t _len = (img_w + _offset + 7)&(~7);\
			MEM_READ1((uint64_t)((ptr + _ptr_limit * img_w)&(~7)), &cache[CC_W * cache_cnt], _len);\
			cache_cnt+=1;} }

#define macro_read_64 {\
			uint64_t _ptr_limit = 0;\
			uint64_t _offset = ((ptr + 0 * img_w) & 7);\
			uint64_t _len = (8 + _offset + 7)&(~7);\
			MEM_READ1((uint64_t)((ptr + _ptr_limit * img_w)&(~7)), &buf8[0], _len);\
			}

#define macro_read_128 {\
			uint64_t _ptr_limit = 0;\
			uint64_t _offset = ((ptr + 0 * img_w) & 7);\
			uint64_t _len = (16 + _offset + 7)&(~7);\
			MEM_READ1((uint64_t)((ptr + _ptr_limit * img_w)&(~7)), &buf16[0], _len);\
			}

THREAD_ENTRY() {
	uint64_t cache_cnt = 0;
	uint8_t cache[CC_W*CC_H];

	uint64_t ptr =   MBOX_GET(resources_sw2rt);
	uint64_t img_w = MBOX_GET(resources_sw2rt);
	uint64_t img_h = MBOX_GET(resources_sw2rt);

	uint8_t buf8[8];
	macro_read_64;
	for(int _i = 0; _i < 8; _i++){
		MBOX_PUT(resources_rt2sw, buf8[_i]);
	}

	uint8_t buf16[16];
	macro_read_128;
	for(int _i = 0; _i < 16; _i++){
		MBOX_PUT(resources_rt2sw, buf16[_i]);
	}

	for(int i = 0; i < 7; i++){
		macro_read_batch;
		MBOX_PUT(resources_rt2sw, cache_cnt-WS);
		
		for(int ii = 0; ii < 8; ii++){
			MBOX_PUT(resources_rt2sw, (uint64_t)(cache[CC_W * (cache_cnt - WS) + ii]));
		}
	}

	// Done
	MBOX_PUT(resources_rt2sw, 0xffffffffffffffff);
	THREAD_EXIT();
}
