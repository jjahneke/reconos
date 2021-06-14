#include "reconos_calls.h"
#include "reconos_thread.h"

THREAD_ENTRY() {
	THREAD_INIT();
	uint64_t buf[4] = {164, 264, 364, 464};

	uint64_t msg = MBOX_GET(rcs1_sw2rt);
	uint64_t ptr = MBOX_GET(rcs1_sw2rt);
	
	MBOX_PUT(rcs1_rt2sw, msg + 64);
	MEM_WRITE(buf, ptr, 4*8);

	MBOX_PUT(rcs1_rt2sw, 0xffffffffffffffff);
}
