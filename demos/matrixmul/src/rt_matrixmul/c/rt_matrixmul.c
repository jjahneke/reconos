#include <limits.h>
#include <stdio.h>

#include "reconos_thread.h"
#include "reconos_calls.h"

#include "../../application/mmp.h"


void std_matrix_mul(int64_t *i_matrix_a, int64_t *i_matrix_b, int64_t *o_matrix_c, int matrix_size) {
	int i, j, k;
	printf("std_matrix_mul i:");fflush(0);
	for (i=0; i<matrix_size; ++i) {
		printf(" %i ", i);fflush(0);
		for (j=0; j<matrix_size; ++j) {
			int64_t temp = 0;
			int pos = i*matrix_size;
			for (k=0; k<matrix_size; ++k) {
				temp += i_matrix_a[pos+k]*i_matrix_b[k*matrix_size+j];
			}
			o_matrix_c[pos+j] = temp;
		}
	}
	printf ("\n");
}

THREAD_ENTRY() {
	uint64_t ret;
	int64_t **ret2;

	while (1) {
		ret = MBOX_GET(resources_address);
		if (ret == 0xffffffffffffffff) {
			THREAD_EXIT();
		}
		ret2 = (int64_t **)ret;
		std_matrix_mul(ret2[0], ret2[1], ret2[2], STD_MMP_MATRIX_SIZE);
		MBOX_PUT(resources_acknowledge, (uint64_t)ret2[2]);
	}
	return;
}
