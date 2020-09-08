/* matrix_functions.h */

#ifndef __MATRIX_FUNCTIONS_H__
#define __MATRIX_FUNCTIONS_H__

void add_matrixes(int64_t* target, int64_t *source_a, int64_t *source_b, int matrix_size);
void sub_matrixes(int64_t* target, int64_t *source_a, int64_t *source_b, int matrix_size);
void copy_matrix(int64_t* target, int64_t *source, int matrix_size);

int read_matrix(int64_t* target, char *source_file, int matrix_size);
int write_matrix(int64_t* source, char *target_file, int matrix_size);

#endif /* __MATRIX_FUNCTIONS_H__ */
