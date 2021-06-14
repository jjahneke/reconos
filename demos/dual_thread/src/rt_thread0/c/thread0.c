#include "reconos_thread.h"
#include "reconos_calls.h"
#include "string.h"

#define MEM_READ1(src, dest, n) memcpy((void*)dest, (void*)src, n)
#define MEM_WRITE1(src, dest, n) memcpy((void*)dest, (void*)src, n)

THREAD_ENTRY() {
	uint64_t buf[4] = {10, 20, 30, 40};

	uint64_t msg = MBOX_GET(rcs0_sw2rt);
	uint64_t ptr = MBOX_GET(rcs0_sw2rt);

	MBOX_PUT(rcs0_rt2sw, msg+42);
	MEM_WRITE1(buf, ptr, 4*8);
	
	MBOX_PUT(rcs0_rt2sw, 0xffffffffffffffff);
	THREAD_EXIT();
}
