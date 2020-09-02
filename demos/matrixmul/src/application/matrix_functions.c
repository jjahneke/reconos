/* matrix_functions.c */

#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>

#include <assert.h>

#include "matrix_functions.h"

void add_matrixes(int64_t* target, int64_t *source_a, int64_t *source_b, int matrix_size) {
	int i;
	for (i=0; i<(matrix_size*matrix_size); ++i) {
		target[i] = source_a[i] + source_b[i];
	}
}

void sub_matrixes(int64_t* target, int64_t *source_a, int64_t *source_b, int matrix_size) {
	int i;
	for (i=0; i<(matrix_size*matrix_size); ++i) {
		target[i] = source_a[i] - source_b[i];
	}
}

void copy_matrix(int64_t* target, int64_t *source, int matrix_size) {
	int i;
	for (i=0; i<(matrix_size*matrix_size); ++i) {
		target[i] = source[i];
	}
}

int read_matrix(int64_t* target, char *source_file, int matrix_size){
	// WARNING: no path string sanitizing! Use with caution!
	FILE* mf = fopen(source_file, "r");

	assert(target != NULL);
	assert(source_file != NULL);

	size_t read_items = fread(target, sizeof(int64_t), matrix_size*matrix_size, mf);
	fclose(mf);

	// Convert from network byte order to host byte order (endianness)
	unsigned long i;
	for(i = 0; i< matrix_size*matrix_size; i++){
		target[i] = ntohl(target[i]);
	}

	return (read_items == (matrix_size*matrix_size) );
}

int write_matrix(int64_t* source, char *target_file, int matrix_size){
	// WARNING: no path string sanitizing! Use with caution!

	// Convert field from host byte order to network byte order (endianness)
	int64_t* source2 = malloc(matrix_size*matrix_size*sizeof(int64_t));
	unsigned long i;
	for(i = 0; i< matrix_size*matrix_size; i++){
		source2[i] = htonl(source[i]);
	}

	// Write field to file
	FILE* mf = fopen(target_file, "w");
	size_t written_items = fwrite(source2, sizeof(int64_t), matrix_size*matrix_size, mf);
	fclose(mf);

	free (source2);
	return (written_items == (matrix_size*matrix_size) );
}
