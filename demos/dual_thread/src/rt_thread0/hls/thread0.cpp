#include "reconos_calls.h"
#include "reconos_thread.h"

THREAD_ENTRY() {
	THREAD_INIT();
	uint64_t buf[4] = {10, 20, 30, 40};

	uint64_t msg = MBOX_GET(rcs0_sw2rt);
	uint64_t ptr = MBOX_GET(rcs0_sw2rt);
	
	MBOX_PUT(rcs0_rt2sw, msg + 42);
	MEM_WRITE(buf, ptr, 4*8);

	MBOX_PUT(rcs0_rt2sw, 0xffffffffffffffff);
}
