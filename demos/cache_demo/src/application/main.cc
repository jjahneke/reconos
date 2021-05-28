#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define WORDSPERLINE 13
#define DWORDSPERLINE 7
#define mask_w0 0xffffffff00000000
#define mask_w1 0x00000000ffffffff

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
}

void print_help() {
	std::cout <<
		"Usage: cachedemo <num_hw_thread> <num_sw_threads> <image>\n"
		"Only use one of the threads, i.e.\n"
		"cachedemo 0 1 <img> *or* cachedemo 1 0 <img>"
	<< std::endl;
}

int main(int argc, char **argv) {
	int num_hwts, num_swts;
	int clk;

	if (argc != 4) {
		print_help();
		return 0;
	}

	num_hwts = atoi(argv[1]);
	num_swts = atoi(argv[2]);

	reconos_init();
	reconos_app_init();
	clk = reconos_clock_threads_set(100000);

	for (int i = 0; i < num_hwts; i++) {
		std::cout << "Creating hw_thread_" << i << std::endl;
		reconos_thread_create_hwt_cachedemo();
	}

	for (int i = 0; i < num_swts; i++) {
		std::cout << "Creating sw_thread_" << i << std::endl;
		reconos_thread_create_swt_cachedemo();
	}
    
	cv::Mat x = cv::imread(argv[3], 0);
	uint8_t* ptr = (uint8_t*)x.data;
	int img_w = x.cols;
	int img_h = x.rows;

	// NOTE_J: 4 Lines 13 WORD (aka 4 Byte) / (7 DWORD) (aka 8 Byte) each
	uint64_t* ret_ptr = (uint64_t*) malloc(4 * DWORDSPERLINE * 8);
	
	mbox_put(resources_sw2rt, (uint64_t)ptr);
	mbox_put(resources_sw2rt, (uint64_t)img_w);
	mbox_put(resources_sw2rt, (uint64_t)img_h);
	mbox_put(resources_sw2rt, (uint64_t)ret_ptr);

	uint64_t _ret;
	uint64_t mask = 0x00000000000000ff;
	for(int i = 0; i < 7; i++){
		std::cout << "Starting batch_read " << i << std::endl;
		_ret = mbox_get(resources_rt2sw);
		std::cout << "Cache_cnt is at " << _ret << std::endl;
		for(int ii = 0; ii < 8; ii++){
			_ret = mbox_get(resources_rt2sw);
			for(int iii = 0; iii < 7; iii++){
				std::cout << (uint16_t)((_ret & (mask << 8*iii)) >> 8*iii) << ", ";
			}
			std::cout << (uint16_t)((_ret & (mask << 8*7)) >> 8*7) << std::endl;
		}
	}

	std::cout << "\n\n";
	for(int i = 0; i < 9; i++){
		_ret = mbox_get(resources_rt2sw);
		for(int iii = 0; iii < 8; iii++){
			std::cout << (uint16_t)((_ret & (mask << 8*iii)) >> 8*iii) << ", ";
		}
		
		_ret = mbox_get(resources_rt2sw);
		for(int iii = 0; iii < 7; iii++){
			std::cout << (uint16_t)((_ret & (mask << 8*iii)) >> 8*iii) << ", ";
		}
		std::cout << (uint16_t)((_ret & (mask << 8*7)) >> 8*7) << std::endl;
	}

	std::cout << "First matrix" << std::endl;
	for(int i = 0; i < 30; i++){
		for(int ii = 0; ii < 30; ii++){
			_ret = mbox_get(resources_rt2sw);
			std::cout << _ret << ", ";
		}
		std::cout << std::endl;
	}

	std::cout << "xfCv matrix" << std::endl;
	for(int i = 0; i < 30; i++){
		for(int ii = 0; ii < 30; ii++){
			_ret = mbox_get(resources_rt2sw);
			std::cout << _ret << ", ";
		}
		std::cout << std::endl;
	}

	for(int i = 0; i < 4; i++){
		_ret = mbox_get(resources_rt2sw);
		std::cout << _ret << std::endl;
		for(int ii = 0; ii < DWORDSPERLINE; ii++){
			uint32_t w0 = (uint32_t)((*(ret_ptr + i*DWORDSPERLINE + ii) & mask_w0) >> 32);
			uint32_t w1 = (uint32_t)(*(ret_ptr + i*DWORDSPERLINE + ii) & mask_w1);
			std::cout << w0 << ", " << w1 << ", ";
		}
		std::cout << std::endl;
	}

	uint64_t ret;
	do{
		ret = mbox_get(resources_rt2sw);
	} while(ret != 0xffffffffffffffff);

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
