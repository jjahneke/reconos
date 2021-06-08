#include "reconos_thread.h"
#include "reconos_calls.h"
#include "string.h"
#include "inttypes.h"
#include "stdio.h"

#define DWORDSPERLINE 7

#define MEM_READ1(src, dest, n) memcpy((void*)dest, (void*)src, n)
#define MEM_WRITE1(src, dest, n) memcpy((void*)dest, (void*)src, n)

void t_fill_remainder(uint64_t mem[7]) {
	uint32_t __mem[8] = {10,20,30,40,50,60,70,80};
	for(int i = 0; i < 4; i++){
		mem[i+3] = (((uint64_t)__mem[2*i]) << 32) | ((uint64_t)__mem[2*i+1]);
	}
}

THREAD_ENTRY() {
	
	uint32_t _mem[5] = {1,2,3,4,5};
	uint64_t mem[7];
	for(int i = 0; i < 2; i++){
		mem[i] = (((uint64_t)_mem[2*i]) << 32) | ((uint64_t)_mem[2*i+1]);
	}
	mem[2] = (((uint64_t)_mem[4]) << 32);
	t_fill_remainder(mem);

	uint64_t ret_ptr = MBOX_GET(rcs_sw2rt);

	for(int i = 0; i < 4; i++){
		MBOX_PUT(rcs_rt2sw, (ret_ptr + i*8*DWORDSPERLINE));
		MEM_WRITE1(mem, (ret_ptr + i*8*DWORDSPERLINE), DWORDSPERLINE*8);
	}

	// Done
	MBOX_PUT(rcs_rt2sw, 0xffffffffffffffff);

	THREAD_EXIT();
}
