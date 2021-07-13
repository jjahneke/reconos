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
const uint64_t BYTEMASK = 0x00000000000000ff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAT_SIZE (WINDOW_SIZE + 6)
#define CACHE_LINES (WINDOW_SIZE * 2)
#define PREFETCH_ROWS (CACHE_LINES)

#define FAST_TH 7
#define MAXPERBLOCK 200
#define DWORDS_KPT 2
#define MASK_W0 0xffffffff00000000
#define MASK_W1 0x00000000ffffffff

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((CC_W - 2*BORDER_EDGE) / WINDOW_SIZE)

void print_help() {
	std::cout <<
		"Usage: fastdemo <sw_or_hw> <img_in>\n"
		"For sw: fastdemo 0 <img_in>\n"
		"For hw: fastdemo 1 <img_in>"
	<< std::endl;
}

int main(int argc, char **argv) {
	int sw_or_hw;
	int clk;

	if (argc != 3) {
		print_help();
		return 0;
	}

	reconos_init();
	reconos_app_init();
	clk = reconos_clock_threads_set(100000);

	sw_or_hw = atoi(argv[1]);
	if(sw_or_hw == 0) { 
		std::cout << "Creating SW Thread" << std::endl;
		reconos_thread_create_swt_fast();
	}
	else { 
		std::cout << "Creating HW Thread" << std::endl;
		reconos_thread_create_hwt_fast();
	}

	std::cout << "Thread creation finished\n";

	uint64_t blocks = NROWS * NCOLS;
	uint64_t mem_size_kpt = MAXPERBLOCK * blocks * DWORDS_KPT;

	cv::Mat img_i = cv::imread(argv[2], 0);
	uint64_t img_w = img_i.cols;
	uint64_t img_h = img_i.rows;
	
	cv::Mat _dummy = cv::Mat::zeros(1,img_w,img_i.type());
	img_i.push_back(_dummy);
	uint8_t* ptr_i = (uint8_t*)img_i.data;
	uint64_t* kpt_ptr = (uint64_t*)calloc(mem_size_kpt, sizeof(uint64_t));
	std::vector<cv::KeyPoint> vToDistributeKeys;
	vToDistributeKeys.reserve(MAXPERBLOCK*NROWS*NCOLS);
	
	
	std::cout << "Preprocessing and memory allocation finished\n";

	// Do thread work
	mbox_put(rcsfast_sw2rt, (uint64_t)ptr_i);
	mbox_put(rcsfast_sw2rt, (uint64_t)img_w);
	mbox_put(rcsfast_sw2rt, (uint64_t)img_h);
	mbox_put(rcsfast_sw2rt, (uint64_t)kpt_ptr);

	std::cout << "Sent data to thread\nWaiting for answer...\n";

	uint64_t ret;
	do {
		ret = mbox_get(rcsfast_rt2sw);
	}
	while(ret != 0xffffffffffffffff);
	
	std::cout << "Answer received, evaluation\n";

	// Construct result vector from memory
	std::ofstream myFile;
	myFile.open("__result.txt", std::ios_base::app);

	uint32_t nfeatures = 0;
	for(unsigned int b = 0; b < blocks; b++){
		uint32_t blockoffset = b * MAXPERBLOCK;
		for(int i = 0; i < MAXPERBLOCK; i++){
			// NOTE_J: DWORD 0: [x, y]
			int x = (int)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 0) & MASK_W0) >> 32);
			int y = (int) (*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 0) & MASK_W1);
			// NOTE_J: DWORD 1: [angle, response]
			float a = (float)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 1) & MASK_W0) >> 32) * std::pow(2,-12);
			float r = (float) (*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 1) & MASK_W1);
			uint32_t id = nfeatures;
			
			// Exit condition for the block, i.e. not all 100 allowed features were filled
			if (x == 0 ||  y == 0){
				break;
			}
			
			if(x >= img_w - 2*BORDER_EDGE || y >= img_h - 2*BORDER_EDGE)
				continue;

			//a = IC_Angle(img, cv::Point2f(x,y));
			myFile << x << ", " << y << ", " << a << ", " << r << ", " << std::endl;
			
			vToDistributeKeys.push_back(cv::KeyPoint((float)x,(float)y,7.,a,r,0,id));
			nfeatures++;
		}
	}
	myFile.close();
	
	std::cout << "KeyPoints found: " << nfeatures << std::endl;

	// Cleanup
	free(kpt_ptr);
	kpt_ptr = NULL;

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
