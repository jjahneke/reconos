extern "C" {
	#include "reconos_thread.h"
	#include "reconos_calls.h"
}

#define BLOCK_SIZE 2048

void bubblesort(uint64_t *data, int data_count) {
	int i;
	uint64_t tmp;
	int s, n, newn;

	s = 1;
	n = data_count - 1;
	newn = n;

	while (s) {
		s = 0;
		for (i = 0; i < n; i++) {
			if (data[i] > data[i + 1]) {
				tmp = data[i];
				data[i] = data[i + 1];
				data[i + 1] = tmp;
				newn = i;
				s = 1;
			}
		}

		n = newn;
	}
}

THREAD_ENTRY() {
	uint64_t ret;

	while (1) {
		ret = MBOX_GET(resources_address);

		if (ret == 0xffffffffffffffff) {
			THREAD_EXIT();
		}

		bubblesort((uint64_t *)ret, BLOCK_SIZE);
		MBOX_PUT(resources_acknowledge, ret);
	}
}
