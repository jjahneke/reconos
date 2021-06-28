#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
}

#define CC_W 1280
#define CC_H 384

void print_help() {
	std::cout <<
		"Usage:  filterdemo <sw_or_hw> <mode> <img_in> <img_out>\n"
		"sw_or_hw: {0: sw, 1: hw}\n"
		"mode: {0: Dirac/Unit, 1: Gaussian 3x3, 1: Sobel X&Y}"
	<< std::endl;
}

int main(int argc, char **argv) {
	int sw_or_hw, mode;
	int clk;

	if (argc != 5) {
		print_help();
		return 0;
	}

	sw_or_hw = atoi(argv[1]);
	mode = atoi(argv[2]);

	reconos_init();
	reconos_app_init();
	clk = reconos_clock_threads_set(100000);

	if(sw_or_hw == 0) { 
		std::cout << "Creating SW Thread" << std::endl;
		reconos_thread_create_swt_filterdemo();
	}
	else { 
		std::cout << "Creating HW Thread" << std::endl;
		return 0;
		//reconos_thread_create_hwt_filterdemo();
	}

	// Open image, allocate output memory
	cv::Mat img_i = cv::imread(argv[3], 0);
	int rows = img_i.rows;
	int cols = img_i.cols;
	// Append zero'd row to compensate last line over-read
	cv::Mat _tmp = cv::Mat::zeros(1,cols,img_i.type());
	img_i.push_back(_tmp);

	cv::Mat img_o = cv::Mat::zeros(rows, cols, CV_8U);
	uint8_t* ptr_i = (uint8_t*)img_i.data;
	uint8_t* ptr_o = (uint8_t*)calloc(CC_W * CC_H, sizeof(uint8_t));

	// Do thread work
	std::cout << "Starting thread work" << std::endl;
	{
		uint64_t ret;

		mbox_put(rcs_sw2rt, (uint64_t)ptr_i);
		mbox_put(rcs_sw2rt, (uint64_t)cols);
		mbox_put(rcs_sw2rt, (uint64_t)rows);
		mbox_put(rcs_sw2rt, (uint64_t)ptr_o);
		mbox_put(rcs_sw2rt, (uint64_t)mode);

		do {
			ret = mbox_get(rcs_rt2sw);
		}
		while(ret != 0xffffffffffffffff);
		std::cout << "Done with thread work!" << std::endl;
	}

	// Create image from allocated memory
	for(int row = 0; row < rows; row++) {
	    uint8_t* row_ptr = (uint8_t*)img_o.ptr(row);
	    memcpy(row_ptr, ptr_o + (row)*CC_W, cols);
	}

	// Store image
	cv::imwrite(argv[4], img_o);
	std::cout << "Image written to " << argv[4] << std::endl;
	
	// Cleanup
	free(ptr_o);
	ptr_o = NULL;

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
