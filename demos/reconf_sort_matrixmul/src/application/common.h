/* common.h */

#ifndef __COMMON_H__
#define __COMMON_H__

#include "mmp.h"

#define PAGE_SIZE 4096

void *xmalloc_aligned(size_t size, size_t alignment);

void generate_data(int64_t *input_matrixes[2], int64_t **output_matrix, int matrix_size);
void generate_result(int64_t *input_matrixes[2], int64_t **res, int matrix_size);

void read_data(int64_t *input_matrixes[2], int64_t **output_matrix, int matrix_size);
void read_result(int64_t **res, int matrix_size);
void write_data(int64_t *input_matrixes[2], int64_t **output_matrix, int matrix_size) ;
void write_result(int64_t **res, int matrix_size);

void print_matrix(int64_t *matrix, char matrix_name, int matrix_size);
int compare_result(int64_t *result, int64_t *compare, int matrix_size);

void append_list(MATRIXES **std_mmp_matrixes, int64_t *i_matrixes[7][3]);
void append_list_single(MATRIXES **str_mmp_matrixes, int64_t *i_matrix);

#endif /* __COMMON_H__ */
