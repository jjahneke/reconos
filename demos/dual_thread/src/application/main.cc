#include <iostream>
#include <string>

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
}

void print_help() {
	std::cout <<
		"Usage: dualthread <sw_or_hw> <en_rt0> <en_rt1>\n"
		"For sw: dualthread 0 <en_rt0> <en_rt1>\n"
		"For hw: dualthread 1 <en_rt0> <en_rt1>"
	<< std::endl;
}

int main(int argc, char **argv) {
	int sw_or_hw, en_rt0, en_rt1; 
	int clk;

	if (argc != 4) {
		print_help();
		return 0;
	}

	sw_or_hw = atoi(argv[1]);
	en_rt0 = atoi(argv[2]);
	en_rt1 = atoi(argv[3]);

	reconos_init();
	reconos_app_init();
	clk = reconos_clock_threads_set(100000);

	if(en_rt0 == 1) {
		std::cout << "Thread 0 enabled!" << std::endl;
		if(sw_or_hw == 0) { 
			std::cout << "Creating sw_thread0" << std::endl;
			reconos_thread_create_swt_thread0();
		}
		else { 
			std::cout << "Creating hw_thread0" << std::endl;
			reconos_thread_create_hwt_thread0();
		}
	}
	if(en_rt1 == 1) {
		std::cout << "Thread 1 enabled!" << std::endl;
		if(sw_or_hw == 0) { 
			std::cout << "Creating sw_thread1" << std::endl;
			reconos_thread_create_swt_thread1();
		}
		else { 
			std::cout << "Creating hw_thread1" << std::endl;
			reconos_thread_create_hwt_thread1();
		}
	}

	if(en_rt0 == 1) {
		uint64_t buf0[4] = {10, 20, 30, 40};
		uint64_t* ptr0 = (uint64_t*)calloc(4, sizeof(uint64_t));
		uint64_t ret0;
		std::cout << "Starting thread_0" << std::endl;

		mbox_put(rcs0_sw2rt, 0);
		mbox_put(rcs0_sw2rt, (uint64_t)ptr0);

		ret0 = mbox_get(rcs0_rt2sw);
		std::cout << "Got answer from thread_0: " << ret0 << std::endl;

		do {
			ret0 = mbox_get(rcs0_rt2sw);
		}
		while(ret0 != 0xffffffffffffffff);
		
		bool valid0 = true;
		std::string s0 = "";
		for(uint64_t i = 0; i < 4; i++) {
			valid0 &= (buf0[i] == *(ptr0+i));
			s0 = s0 + std::to_string(*(ptr0+i)) + " ";
		}
		std::cout << s0 << std::endl;

		free(ptr0);
		ptr0 = NULL;

		std::cout << "Done with thread_0, mem is valid? " << valid0 << std::endl;
	}

	if(en_rt1 == 1) {
		uint64_t buf1[4] = {164, 264, 364, 464};
		uint64_t* ptr1 = (uint64_t*)calloc(4, sizeof(uint64_t));
		uint64_t ret1;
		std::cout << "Starting thread_1" << std::endl;

		mbox_put(rcs1_sw2rt, 0);
		mbox_put(rcs1_sw2rt, (uint64_t)ptr1);

		ret1 = mbox_get(rcs1_rt2sw);
		std::cout << "Got answer from thread_1: " << ret1 << std::endl;

		do {
			ret1 = mbox_get(rcs1_rt2sw);
		}
		while(ret1 != 0xffffffffffffffff);
		
		bool valid1 = true;
		std::string s1 = "";
		for(uint64_t i = 0; i < 4; i++) {
			valid1 &= (buf1[i] == *(ptr1+i));
			s1 = s1 + std::to_string(*(ptr1+i)) + " ";
		}
		std::cout << s1 << std::endl;

		free(ptr1);
		ptr1 = NULL;
		std::cout << "Done with thread_1, mem is valid? " << valid1 << std::endl;
	}

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
