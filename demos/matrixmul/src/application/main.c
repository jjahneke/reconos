/* matrixmul.c */
#define  _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"
#include "timer.h"

#include "mmp.h"
#include "common.h"
#include "main.h"

#define log(...) printf(__VA_ARGS__); fflush(stdout)


// mailboxes
struct mbox mb_start;
struct mbox mb_stop;

void start_threads_hw (int hwt_count)
{
	int i;
	printf("Creating %i hw-threads: ", hwt_count);
	fflush(stdout);
	for (i = 0; i < hwt_count; i++)
	{
		printf(" %i",i);fflush(stdout);
		reconos_thread_create_hwt_matrixmul();
	}
	printf("\n");
}

void start_threads_sw (int swt_count)
{
	int i;
	printf("Creating %i sw-threads: ", swt_count);
	fflush(stdout);
	for (i = 0; i < swt_count; i++) {
		printf(" %i", i);
		fflush(stdout);
		reconos_thread_create_swt_matrixmul();
	 }
	printf("\n");
}

void print_help() {
	printf("\n"
	       "ReconOS v4 matrix multiplication application\n"
	       "--------------------------------\n"
	       "\n"
	       "Multiplies two matrices with a variable number of sw and hw threads using the Strassen algorithm.\n"
	       "\n"
	       "Usage:\n"
	       "    matrixmul <num_hw_threads> <num_sw_threads> <size_of_matrix>\n"
	       "\n"
	       "    <num_hw_threads> - Number of hardware threads to create. The maximum number is\n"
	       "                       limited by the hardware design.\n"
	       "    <num_sw_threads> - Number of software threads to create.\n"
	       "    <num_of_blocks>  - Size of matrices to multiply. Must be one of 256,512,1024 or 2048\n"
	       "\n"
	);
}

/*
 * MAIN
 */
int main(int argc, char **argv) {
	int i;

	unsigned generate_data_time;
	unsigned generate_check_result_time;
	unsigned init_hwt_time;
	unsigned init_swt_time;
	unsigned str_mmp_split;
	unsigned std_mmp_time;
	unsigned str_mmp_combine;
	unsigned terminate_hwt_time;
	unsigned terminate_swt_time;
	unsigned comparision_time;
	unsigned calculation_time_std;
	unsigned calculation_time_str;
	
	printf("matrixmul build: %s %s\n", __DATE__, __TIME__);
	printf("Initializing reconos...\n");
	reconos_init();
	reconos_app_init();
	timer_init();
	
	if (argc != 4) {
		print_help();
		return 0;
	}

	int hw_threads = atoi(argv[1]);
	int sw_threads = atoi(argv[2]);
	int str_matrix_size = atoi(argv[3]);

	int std_matrix_size	= STD_MMP_MATRIX_SIZE; // Fixed by hardware thread


	int mbox_size = (int) pow(7, ((int)log2(str_matrix_size)) - ((int)log2(STD_MMP_MATRIX_SIZE)));
	printf("Size of mailboxes: %i\n", mbox_size);
	printf("Address of mb_start: %p\n", &mb_start);

	int *i_matrixes[2]	= {NULL, NULL};
	int *o_matrix		= NULL;
	int *compare		= NULL;

	MATRIXES* std_mmp_matrixes = NULL;

	log("Generating input data.\n");
	generate_data_time = timer_get();
	generate_data(i_matrixes, &o_matrix, str_matrix_size);
	generate_data_time = timer_get() - generate_data_time;

	log("Generating check results.\n");
	generate_check_result_time = timer_get();
	generate_result(i_matrixes, &compare, str_matrix_size);
	generate_check_result_time = timer_get() - generate_check_result_time;

	init_hwt_time = timer_get();

	// init hw-threads
	log("Creating %i hw-thread(s).\n", hw_threads);

	start_threads_hw(hw_threads);
	
	init_hwt_time = timer_get() - init_hwt_time;

	// init sw-threads
	log("Creating %i software threads.\n", sw_threads);
	init_swt_time = timer_get();
	start_threads_sw(sw_threads);
	init_swt_time = timer_get() - init_swt_time;

	// split input matrixes recursively (strassen algorithm part 1)
	log("Running Strassen algorithm part 1 - split.\n");
	str_mmp_split = timer_get();
	str_matrix_split(i_matrixes[0], i_matrixes[1], &std_mmp_matrixes, str_matrix_size);
	str_mmp_split = timer_get() - str_mmp_split;

	// calculate matrixes with standard mmp algorithm (in hw and/or sw)
	log("Putting matrix pointers in mbox.\n");
	std_mmp_time = timer_get();
	MATRIXES *ptr = std_mmp_matrixes;

	for (i=0; i<mbox_size; ++i) {
		printf("Putting pointer to matrixes into mbox: %p, %p, %p\n", ptr->matrixes[0],ptr->matrixes[1],ptr->matrixes[2]);
		mbox_put(resources_address,(unsigned int)(ptr->matrixes));
		ptr = ptr->next;
	}
	log("Waiting for acknowledgements...\n");
	for (i=0; i<mbox_size; ++i) {
		printf("Getting pointer to matrixes from mbox: %p\n", (void*)mbox_get(resources_acknowledge));
	}
	std_mmp_time = timer_get() - std_mmp_time;
	log("Got acknowledgments.\n");

	// terminate threads
	log("Sending terminate message to %i thread(s).\n", hw_threads + sw_threads);
	terminate_hwt_time = timer_get();
	terminate_swt_time = terminate_hwt_time;
	for (i = 0; i < hw_threads + sw_threads; i++) {
		log("Putting a stop message into MBOX...\n");
		mbox_put(resources_address,UINT_MAX);
	}

	// well, this will now measure termination time of all threads....
	terminate_hwt_time = timer_get() - terminate_hwt_time;
	terminate_swt_time = timer_get() - terminate_swt_time;

	log("Threads have been terminated.\n");

	// combine results (strassen algorithm part 2)
	log("Running Strassen algorithm part 2 - combine.\n");
	str_mmp_combine = timer_get();
	o_matrix = str_matrix_combine(&std_mmp_matrixes, std_matrix_size, str_matrix_size);
	str_mmp_combine = timer_get() - str_mmp_combine;

	// check, if results are correct
	comparision_time = timer_get();
	int correct_result =  compare_result(o_matrix, compare, str_matrix_size);
	comparision_time = timer_get() - comparision_time;

	if (correct_result == -1) {
		log("\nResult is correct.\n\n");
	} else {
		log("\nBad result.\n");
		printf("Comparison failed at index %i.Correct: %i, Actual result: %i.\n", correct_result, compare[correct_result], o_matrix[correct_result] );
#if 1
		print_matrix(i_matrixes[0], 'A', str_matrix_size);
		print_matrix(i_matrixes[1], 'B', str_matrix_size);
		print_matrix(o_matrix    , 'C', str_matrix_size);
		print_matrix(compare          , 'Z', str_matrix_size);
#endif
		log("\n");
		exit(EXIT_FAULTY_RESULT);
	}

	calculation_time_std = generate_check_result_time;
	calculation_time_str = init_hwt_time + init_swt_time + str_mmp_split + std_mmp_time + terminate_swt_time + str_mmp_combine;

	log("Timing information\n");
	log("==================\n");
	log("Generate input data:   %f ms\n", timer_toms(generate_data_time));
	log("Generate check result: %f ms\n", timer_toms(generate_check_result_time));
	log("Initializing HWT:      %f ms\n", timer_toms(init_hwt_time));
	log("Initializing SWT:      %f ms\n", timer_toms(init_swt_time));
	log("Str. split (part 1):   %f ms\n", timer_toms(str_mmp_split));
	log("Std. MMP:              %f ms\n", timer_toms(std_mmp_time));
	log("Str. combine (part 2): %f ms\n", timer_toms(str_mmp_combine));
	log("~Thread term. HWT:     %f ms\n", timer_toms(terminate_hwt_time));
	log("~Thread term. SWT:     %f ms\n", timer_toms(terminate_swt_time));
	log("Check HWT result:      %f ms\n\n", timer_toms(comparision_time));
	log("Important timing results\n");
	log("========================\n");
	log("Runtime Std. MMP:      %f ms\n", timer_toms(calculation_time_std));
	log("Runtime Str. MMP:      %f ms\n", timer_toms(calculation_time_str));

	
	timer_cleanup();
	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}
