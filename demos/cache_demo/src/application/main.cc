#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define WORDSPERLINE 13
#define DWORDSPERLINE 7
#define FAST_WS 30

#define mask_w0 0xffffffff00000000
#define mask_w1 0x00000000ffffffff

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
}

void print_help() {
	std::cout <<
		"Usage: cachedemo <sw_or_hw> <image>\n"
		"For sw: cachedemo 0 <image>\n"
		"For hw: cachedemo 1 <image>"
	<< std::endl;
}

int main(int argc, char **argv) {
	int sw_or_hw;
	int clk;

	if (argc != 3) {
		print_help();
		return 0;
	}

	sw_or_hw = atoi(argv[1]);

	reconos_init();
	reconos_app_init();
	clk = reconos_clock_threads_set(100000);

	if(sw_or_hw == 0) { 
		std::cout << "Creating sw_thread cachedemo" << std::endl;
		reconos_thread_create_swt_cachedemo();
	}
	else { 
		std::cout << "Creating hw_thread cachedemo" << std::endl;
		reconos_thread_create_hwt_cachedemo();
	}
		
	cv::Mat x = cv::imread(argv[2], 0);
	uint8_t* ptr = (uint8_t*)x.data;
	int img_w = x.cols;
	int img_h = x.rows;

	// NOTE_J: 4 Lines 13 WORD (aka 4 Byte) / (7 DWORD) (aka 8 Byte) each
	uint64_t* ret_ptr = (uint64_t*) malloc(4 * DWORDSPERLINE * 8);
	uint64_t* ret_ptr2 = (uint64_t*) calloc(2, sizeof(uint64_t));
	
	mbox_put(resources_sw2rt, (uint64_t)ptr);
	mbox_put(resources_sw2rt, (uint64_t)img_w);
	mbox_put(resources_sw2rt, (uint64_t)img_h);
	mbox_put(resources_sw2rt, (uint64_t)ret_ptr);
	mbox_put(resources_sw2rt, (uint64_t)ret_ptr2);

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
	for(int i = 0; i < FAST_WS; i++){
		for(int ii = 0; ii < FAST_WS; ii++){
			_ret = mbox_get(resources_rt2sw);
			std::cout << _ret << ", ";
		}
		std::cout << std::endl;
	}

	std::cout << "xfCv/Dummy matrix" << std::endl;
	for(int i = 0; i < FAST_WS; i++){
		for(int ii = 0; ii < FAST_WS; ii++){
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

	std::cout << "Second mem test, uint8_t array via MEM_WRITE" << std::endl;
	for(int i = 0; i < 16; i++) {
		std::cout << (short)*((uint8_t*)ret_ptr2 + i) << ", ";
	}
	std::cout << std::endl;

	uint64_t ret;
	do{
		ret = mbox_get(resources_rt2sw);
	} while(ret != 0xffffffffffffffff);

	free(ret_ptr);
	ret_ptr = NULL;	
	free(ret_ptr2);
	ret_ptr2 = NULL;	

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
