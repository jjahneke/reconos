#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
}

int main(int argc, char **argv) {
	int num_hwts, num_swts;
	int clk;

	if (argc != 3)
		return 0;

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
    
	uint64_t cache_cnt = 0;
	
	cv::Mat x = cv::imread("/home/linaro/configOS2/01/image_0/000000.png", 0);
	uint8_t* ptr = (uint8_t*)x.data;
	int img_w = x.cols;
	int img_h = x.rows;
	
	mbox_put(resources_sw2rt, (uint64_t)ptr);
	mbox_put(resources_sw2rt, (uint64_t)img_w);
	mbox_put(resources_sw2rt, (uint64_t)img_h);

	uint64_t _ret;
	std::cout << "Eval buf[8]" << std::endl;
	for(int _i = 0; _i < 8; _i++){
		_ret = mbox_get(resources_rt2sw);
		std::cout << "Pixel " << _i << ": " << _ret << std::endl;
	}

	std::cout << "Eval buf[16]" << std::endl;
	for(int _i = 0; _i < 16; _i++){
		_ret = mbox_get(resources_rt2sw);
		std::cout << "Pixel " << _i << ": " << _ret << std::endl;
	}

	for(int i = 0; i < 7; i++){
		std::cout << "Starting batch_read " << i << std::endl;
		_ret = mbox_get(resources_rt2sw);
		std::cout << "Cache_cnt is at " << _ret << std::endl;
		for(int ii = 0; ii < 8; ii++){
			_ret = mbox_get(resources_rt2sw);
			std::cout << "Pixel " << ii << ": " << _ret << std::endl;
		}
	}

	uint64_t ret;
	do{
		ret = mbox_get(resources_rt2sw);
	} while(ret != 0xffffffffffffffff);

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
