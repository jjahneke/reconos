#include "reconos_calls.h"
#include "reconos_thread.h"

//#include "own/helper.hpp"

#define DWORDSPERLINE 7

void fill_remainder(uint64_t mem[7]) {
	uint32_t __mem[8] = {10,20,30,40,50,60,70,80};
	for(int i = 0; i < 4; i++){
		mem[i+3] = (((uint64_t)__mem[2*i]) << 32) | ((uint64_t)__mem[2*i+1]);
	}
}

THREAD_ENTRY() {
	THREAD_INIT();
	
	uint32_t _mem[5] = {1,2,3,4,5};
	uint64_t mem[7];
	for(int i = 0; i < 2; i++){
		mem[i] = (((uint64_t)_mem[2*i]) << 32) | ((uint64_t)_mem[2*i+1]);
	}
	mem[2] = (((uint64_t)_mem[4]) << 32);
	fill_remainder(mem);

	uint64_t ret_ptr = MBOX_GET(rcs_tsw2rt);
	
	for(int i = 0; i < 4; i++){
		MEM_WRITE(mem, (ret_ptr + i*8*DWORDSPERLINE), DWORDSPERLINE*8);
	}

	// Done
	MBOX_PUT(rcs_tsw2rt, 0xffffffffffffffff);
}
