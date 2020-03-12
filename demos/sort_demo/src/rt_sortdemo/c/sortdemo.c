#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048

void bubblesort(uint32_t *data, int data_count) {
	int i;
	uint32_t tmp;
	int s, n, newn;

	uint32_t *data2 = (uint32_t*) (((uint64_t) (0x0000007F00000000)) | ((uint64_t) data));

	s = 1;
	n = data_count - 1;
	newn = n;

	while (s) {
		s = 0;
		for (i = 0; i < n; i++) {
			if (data2[i] > data2[i + 1]) {
				tmp = data2[i];
				data2[i] = data2[i + 1];
				data2[i + 1] = tmp;
				newn = i;
				s = 1;
			}
		}

		n = newn;
	}
}

THREAD_ENTRY() {
	uint32_t ret;

	while (1) {
		ret = MBOX_GET(resources_address);

		if (ret == 0xffffffff) {
			THREAD_EXIT();
		}

		bubblesort((uint32_t *)ret, BLOCK_SIZE);
		MBOX_PUT(resources_acknowledge, ret);
	}
}
