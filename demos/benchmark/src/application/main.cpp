//ReconOS runtime
#ifdef __cplusplus
extern "C" {
#endif

#include "reconos.h"
#include "reconos_app.h"
#include "timer.h"

#ifdef __cplusplus
}
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "pwr_monitor.h"

#define log(...) printf(__VA_ARGS__); fflush(stdout)

#define NUM_BLOCKSIZES 14

void print_help() {
	printf("\n"
	       "ReconOS 64bit benchmark application\n"
	       "\n"
	       "Tests MEMIF performance.\n"
	       "\n"
	       "Usage:\n"
	       "    benchmark <num_hw_threads> <num_of_blocks>\n"
	       "\n"
	       "    <num_hw_threads> - Number of hardware threads to create. The maximum number is\n"
	       "                       limited by the hardware design.\n"
	       "    <thread_clk_setting>  - Thread frequency in MHz.\n"
		   "    <num_iterations>  - Must be power of 2\n"
	       "\n"
	);
}

int main(int argc, char **argv) {
	int i;
	int num_hwts, thread_clk_setting, num_iterations;

	if (argc != 4) {
		print_help();
		return 0;
	}

	num_hwts = atoi(argv[1]);
	thread_clk_setting = atoi(argv[2]);
	num_iterations = atoi(argv[3]); //must be power of 2

	unsigned int t_start, t_gen, t_check;
	unsigned int t_transfer_to_pl[NUM_BLOCKSIZES];
	unsigned int t_transfer_from_pl[NUM_BLOCKSIZES];
	unsigned int t_transfer_tofrom_pl[NUM_BLOCKSIZES];

	uint64_t cycles_transfer_to_pl[num_hwts][NUM_BLOCKSIZES];
	uint64_t cycles_transfer_from_pl[num_hwts][NUM_BLOCKSIZES];

	reconos_init();
	reconos_app_init();
	timer_init();

	int clk_set = reconos_clock_threads_set(thread_clk_setting*1000)/1000;
	log("Set clock to %d MHz\n", clk_set);

	log("creating %d hw-threads: \n", num_hwts);
	for (i = 0; i < num_hwts; i++) {
		log("hw-thread %d\n", i);
		reconos_thread_create_hwt_bram_test();
	}
	log("\n");

	log("Starting power monitoring into file..\n");
	std::string log_file = "pwr.log";
	power_monitoring_start(log_file, 2); //logs PS+PL power every second

    int blocksizes_words[NUM_BLOCKSIZES] = {1, 8, 16, 32, 64, 128, 512, 1024, 2048, 4096, 8192, 16384, 32768, 49152}; //limited by BRAM size
	uint64_t* data_pointers[num_hwts][NUM_BLOCKSIZES];
	uint64_t* copy_pointers[num_hwts][NUM_BLOCKSIZES];

	t_start = timer_get();
	log("generating data ...\n");

	for(int i=0; i<NUM_BLOCKSIZES; i++)
	{
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			data_pointers[hwt][i] = (uint64_t*) aligned_alloc(8, blocksizes_words[i] * sizeof(uint64_t));
			for(int e=0; e<blocksizes_words[i]; e++)
			{
				data_pointers[hwt][i][e] = 0x00ff000000000000 + e;
			}

			//make a copy
			copy_pointers[hwt][i] = (uint64_t*) malloc(blocksizes_words[i] * sizeof(uint64_t));
			memcpy(copy_pointers[hwt][i], data_pointers[hwt][i], blocksizes_words[i] * sizeof(uint64_t));
		}
	}
	t_gen = timer_get() - t_start;
	log("generate complete (took %f ms)\n", timer_toms(t_gen));

	for(int i=0; i<NUM_BLOCKSIZES; i++)
	{
		log("running test for blocksize %d B..\n", blocksizes_words[i]*8);
		t_start = timer_get();

		//initiate transfers for all HWTs (immediately AFTER each other..)
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			mbox_put(resources_address, (uint64_t) data_pointers[hwt][i]);
			//mbox_put(resources_length, (uint64_t) blocksizes_words[i]*8); //in bytes
			//log("sent addr\n");
		}

		//send length mboxes separately to avoid deadlock b/w HWTs
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			mbox_put(resources_length, (uint64_t) blocksizes_words[i]*8); //in bytes
			//log("sent len\n");
		}
		
		//send number of iterations to be run
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			mbox_put(resources_iterations, (uint64_t) log2(num_iterations)); //in log(2) -> 1 = 1 iteration, 10 = 1024 iterations. HWT will report avg runtimes
		}

		//HWTs load data into BRAM and writes it back to original address

		//wait for read transfer completion of all HWTs (note: we cannot distinguish which HWT/slot sent the ack!)
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			cycles_transfer_to_pl[hwt][i] = mbox_get(resources_acknowledge_read);
			//log("got an ack\n");
		}

		//t_transfer_to_pl[i] = timer_get() - t_start;
		//t_start = timer_get();

				//wait for read transfer completion of all HWTs (note: we cannot distinguish which HWT/slot sent the ack!)
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			cycles_transfer_from_pl[hwt][i] = mbox_get(resources_acknowledge_write);
			//log("got an ack\n");
		}

		//t_transfer_from_pl[i] = timer_get() - t_start;
		t_transfer_tofrom_pl[i] = timer_get() - t_start;
	}

	//int error_flag = 0;
	t_start = timer_get();
	log("checking data...\n");
	for(int i=0; i<NUM_BLOCKSIZES; i++)
	{
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			for(int e=0; e<blocksizes_words[i]; e++)
			{
				if(data_pointers[hwt][i][e] != copy_pointers[hwt][i][e])
				{
					log("ERROR: Data mismatch in hwt %d, block %d, element %d: %.16lx should be %.16lx\n", hwt, i, e, data_pointers[hwt][i][e], copy_pointers[hwt][i][e]);
					//error_flag = 1;
					return 0;
				}
			}
		}
	}
	t_check = timer_get() - t_start;
	log("check complete (took %f ms)\n", timer_toms(t_check));

	//if(error_flag) return;

	log("terminating HWTs..\n");
	for (i = 0; i < num_hwts; i++) 
	{
		mbox_put(resources_address, 0xffffffffffffffff);
	}

	log("cleaning up ReconOS..\n");
	timer_cleanup();
	reconos_app_cleanup();
	reconos_cleanup();

	//calculation
	double hwt_frequency_mhz = clk_set;

	double transfer_time_to_pl[num_hwts][NUM_BLOCKSIZES];   //ms
	double transfer_time_from_pl[num_hwts][NUM_BLOCKSIZES]; //ms
	double throughput_to_pl[num_hwts][NUM_BLOCKSIZES];      //MB/s
	double throughput_from_pl[num_hwts][NUM_BLOCKSIZES];    //MB/s
	double throughput_to_pl_avg[NUM_BLOCKSIZES];            //MB/s
	double throughput_from_pl_avg[NUM_BLOCKSIZES];          //MB/s
	double throughput_to_pl_total[NUM_BLOCKSIZES];          //MB/s
	double throughput_from_pl_total[NUM_BLOCKSIZES];        //MB/s
	double throughput_to_pl_sw_total[NUM_BLOCKSIZES];       //MB/s
	double throughput_from_pl_sw_total[NUM_BLOCKSIZES];     //MB/s
	double throughput_tofrom_pl_sw_total[NUM_BLOCKSIZES];     //MB/s

	double throughput_to_pl_total_peak = 0; //MB/s
	double throughput_from_pl_total_peak = 0; //MB/s

	for(int i=0; i<NUM_BLOCKSIZES; i++)
	{
		throughput_to_pl_total[i] = 0;
		throughput_from_pl_total[i] = 0;

		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			transfer_time_to_pl[hwt][i] = cycles_transfer_to_pl[hwt][i] / (hwt_frequency_mhz*1000000) *1000;
			transfer_time_from_pl[hwt][i] = cycles_transfer_from_pl[hwt][i] / (hwt_frequency_mhz*1000000) *1000;

			throughput_to_pl[hwt][i] = blocksizes_words[i]*8 / (transfer_time_to_pl[hwt][i]/1000) /1000 /1000;
			throughput_from_pl[hwt][i] = blocksizes_words[i]*8 / (transfer_time_from_pl[hwt][i]/1000) /1000 /1000;

			throughput_to_pl_total[i] += throughput_to_pl[hwt][i];
			throughput_from_pl_total[i] += throughput_from_pl[hwt][i];
		}

		throughput_to_pl_avg[i] = throughput_to_pl_total[i] / num_hwts;
		throughput_from_pl_avg[i] = throughput_from_pl_total[i] / num_hwts;

		//throughput_to_pl_sw_total[i] = num_hwts*blocksizes_words[i]*8 / (timer_toms(t_transfer_to_pl[i])/1000) /1000 /1000;
		//throughput_from_pl_sw_total[i] = num_hwts*blocksizes_words[i]*8 / (timer_toms(t_transfer_from_pl[i])/1000) /1000 /1000;
		throughput_tofrom_pl_sw_total[i] = num_iterations*2*num_hwts*blocksizes_words[i]*8 / (timer_toms(t_transfer_tofrom_pl[i])/1000) /1000 /1000;

		if(throughput_to_pl_total[i] > throughput_to_pl_total_peak) throughput_to_pl_total_peak = throughput_to_pl_total[i];
		if(throughput_from_pl_total[i] > throughput_from_pl_total_peak) throughput_from_pl_total_peak = throughput_from_pl_total[i];
	}

	log("Writing measurement results to file..\n");

	time_t rawtime;
  	struct tm * timeinfo;
  	time (&rawtime);
  	timeinfo = localtime (&rawtime);

	char timestamp [64];
	strftime (timestamp, 64, "%d.%m.%y_%H%M", timeinfo);

	char filename [128];
  	sprintf (filename, "benchmark_run_%dHWT_%dMHz_%s.txt", num_hwts, clk_set, timestamp);

	FILE* logfile;
	logfile = fopen(filename, "w+");

	//fprintf (logfile, "--- Logfile generated at %s ---\n", timestamp);
	//fprintf (logfile, "%d HWTs clocked @ %d MHz running %d transfers per blocksize\n", num_hwts, clk_set, num_iterations);

	for(int i=0; i<NUM_BLOCKSIZES; i++)
	{
		if(blocksizes_words[i]*8 >= 1024)
		{
			log("--- Transfer block size: %d KiB ---\n", blocksizes_words[i]*8/1024);
			//fprintf(logfile, "--- Transfer block size: %d KiB ---\n", blocksizes_words[i]*8/1024);
		}
		else
		{
			log("--- Transfer block size: %d B ---\n", blocksizes_words[i]*8);
			//fprintf(logfile, "--- Transfer block size: %d B ---\n", blocksizes_words[i]*8);
		}
		
		//per HWT (cycle-based)
		for(int hwt=0; hwt<num_hwts; hwt++)
		{
			log("   HWT %d: read: %f ms - %.2f MB/s\n", hwt, transfer_time_to_pl[hwt][i], throughput_to_pl[hwt][i]);
			log("           write: %f ms - %.2f MB/s\n", transfer_time_from_pl[hwt][i], throughput_from_pl[hwt][i]);

			//fprintf(logfile, "   HWT %d: read: %f ms - %.2f MB/s\n", hwt, transfer_time_to_pl[hwt][i], throughput_to_pl[hwt][i]);
			//fprintf(logfile, "           write: %f ms - %.2f MB/s\n", transfer_time_from_pl[hwt][i], throughput_from_pl[hwt][i]);
		}

		//average (cycle-based)
		log("Read -> Average throughput: %.2f MB/s - Total throughput: %.2f MB/s\n", throughput_to_pl_avg[i], throughput_to_pl_total[i]);
		log("Write -> Average throughput: %.2f MB/s - Total throughput: %.2f MB/s\n", throughput_from_pl_avg[i], throughput_from_pl_total[i]);

		//fprintf(logfile, "Read -> Average throughput: %.2f MB/s - Total throughput: %.2f MB/s\n", throughput_to_pl_avg[i], throughput_to_pl_total[i]);
		//fprintf(logfile, "Write -> Average throughput: %.2f MB/s - Total throughput: %.2f MB/s\n", throughput_from_pl_avg[i], throughput_from_pl_total[i]);
		//sanity check (timer-based)
		//log("SW timer (incl. overhead) for all HWTs: read: %f ms - %.2f MB/s\n", timer_toms(t_transfer_to_pl[i]), throughput_to_pl_sw_total[i]);
		//log("SW timer (incl. overhead) for all HWTs: write: %f ms - %.2f MB/s\n", timer_toms(t_transfer_from_pl[i]), throughput_from_pl_sw_total[i]);
		log("Sanity check: SW timer (incl. overhead) for all HWTs: read/write average: %f ms - %.2f MB/s\n", timer_toms(t_transfer_tofrom_pl[i]/(num_iterations*2)), throughput_tofrom_pl_sw_total[i]);

		//fprintf(logfile, "SW timer (incl. overhead) for all HWTs: read: %f ms - %.2f MB/s\n", timer_toms(t_transfer_to_pl[i]), throughput_to_pl_sw_total[i]);
		//fprintf(logfile, "SW timer (incl. overhead) for all HWTs: write: %f ms - %.2f MB/s\n", timer_toms(t_transfer_from_pl[i]), throughput_from_pl_sw_total[i]);
	}

	//fprintf(logfile, "-------\nTotal throughput peak: Read: %.0f MB/s, Write: %.0f MB/s\n", throughput_to_pl_total_peak, throughput_from_pl_total_peak);

	//write table 1
	fprintf(logfile, "%d HWTs | @ %d MHz | %d transfers per block, Block size, Block size [B], avg throughput read [MB/s], avg throughput write [MB/s], total throughput read [MB/s], total throughput write [MB/s]\n", num_hwts, clk_set, num_iterations);
	for(int i=0; i<NUM_BLOCKSIZES; i++)
	{
		if(blocksizes_words[i]*8 >= 1024)
			fprintf(logfile, ",%d KiB,%d,%.2f,%.2f,%.2f,%.2f\n", blocksizes_words[i]*8/1024, blocksizes_words[i]*8, throughput_to_pl_avg[i], throughput_from_pl_avg[i], throughput_to_pl_total[i], throughput_from_pl_total[i]);
		else
			fprintf(logfile, ",%d B,%d,%.2f,%.2f,%.2f,%.2f\n", blocksizes_words[i]*8, blocksizes_words[i]*8, throughput_to_pl_avg[i], throughput_from_pl_avg[i], throughput_to_pl_total[i], throughput_from_pl_total[i]);
		
	}

	fclose(logfile);

	log("Stopping power monitoring..\n");
	power_monitoring_stop(2);

	return 1;
}
