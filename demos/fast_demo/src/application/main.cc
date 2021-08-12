#include <iostream>
#include <cstring>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
}

#ifdef __aarch64__ // ReconOS64
	#define R64 1
    #define BASETYPE uint64_t
    #define BYTES 8
    #define MASK 7
	#define DWORDS_KPT 1
	#define DONEFLAG 0xffffffffffffffff
#else // ReconOS32
	#define R64 0
    #define BASETYPE uint32_t
    #define BYTES 4
    #define MASK 3
	#define DWORDS_KPT 2
	#define DONEFLAG 0xffffffff
#endif
	
// Only used by R64
#define MASK_W0 0xffff000000000000
#define MASK_W1 0x0000ffff00000000
#define MASK_W2 0x00000000ffff0000
#define MASK_W3 0x000000000000ffff

// Only used by R32
#define MASK_S0 0xffff0000
#define MASK_S1 0x0000ffff

#define MAX_W 640 // In pixel
#define BYTEPERPIXEL 3
#define CC_W (MAX_W*BYTEPERPIXEL) // In Byte
#define CC_H 480
const BASETYPE BYTEMASK = 0xff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAXPERBLOCK 200

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((MAX_W - 2*BORDER_EDGE) / WINDOW_SIZE)

void print_help() {
	std::cout <<
		"Usage: fastdemo <sw_or_hw> <img_in> <img_out>\n"
		"For sw: fastdemo 0 <img_in> <img_out>\n"
		"For hw: fastdemo 1 <img_in> <img_out>"
	<< std::endl;
}

int main(int argc, char** argv) {
	if(argc != 4) {
		print_help();
		return 0;
	}
	
	reconos_init();
	reconos_app_init();
	int clk = reconos_clock_threads_set(100000);

	int sw_or_hw = atoi(argv[1]);
	if(sw_or_hw == 0) { 
		std::cout << "Creating SW Thread" << std::endl;
		reconos_thread_create_swt_fast();
	}
	else { 
		std::cout << "Creating HW Thread" << std::endl;
		reconos_thread_create_hwt_fast();
	}

	std::cout << "Thread creation finished\n";
	BASETYPE blocks = NROWS * NCOLS;
	BASETYPE mem_size_kpt = MAXPERBLOCK * blocks * DWORDS_KPT;

	cv::Mat img_i = cv::imread(argv[2]);
	cv::Mat img_gray = cv::imread(argv[2], 0);
	BASETYPE img_w = img_i.cols;
	BASETYPE img_h = img_i.rows;
	
	//cv::Mat _dummy = cv::Mat::zeros(2,img_w,img_i.type());
	//img_i.push_back(_dummy);
	uint8_t* ptr_i = (uint8_t*)img_i.data;
	//BASETYPE* kpt_ptr = (BASETYPE*)calloc(mem_size_kpt, sizeof(BASETYPE));
	BASETYPE* kpt_ptr = (BASETYPE*)malloc(mem_size_kpt * sizeof(BASETYPE));
	memset(kpt_ptr, 0, mem_size_kpt * sizeof(BASETYPE));

	std::vector<cv::KeyPoint> vToDistributeKeys;
	vToDistributeKeys.reserve(MAXPERBLOCK*NROWS*NCOLS);

	// Thread computations
	mbox_put(rcsfast_sw2rt, (BASETYPE)ptr_i);
	mbox_put(rcsfast_sw2rt, (BASETYPE)kpt_ptr);
	mbox_put(rcsfast_sw2rt, (BASETYPE)img_w);
	mbox_put(rcsfast_sw2rt, (BASETYPE)img_h);
	
	std::cout << "Sent data to thread\nWaiting for answer...\n";

	BASETYPE ret;
	do {
		ret = mbox_get(rcsfast_rt2sw);
	}
	while(ret != DONEFLAG);

	std::ofstream myFile;
	myFile.open("__result.txt", std::ios_base::trunc);
	// Construct result vector from memory
	uint32_t nfeatures = 0;
	for(unsigned int b = 0; b < blocks; b++){
		uint32_t blockoffset = b * MAXPERBLOCK;
		for(int i = 0; i < MAXPERBLOCK; i++){
			int x,y;
			float a,r;
			if(R64) {
				x = (int)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT) & MASK_W0) >> 48);
				y = (int)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT) & MASK_W1) >> 32);
				a = (float)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT) & MASK_W2) >> 16) * std::pow(2,-12);
				r = (float)(*(kpt_ptr + (blockoffset + i)*DWORDS_KPT) & MASK_W3);
			}
			else {
				x = (int)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 0) & MASK_S0) >> 16);
				y = (int)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 0) & MASK_S1));
				a = (float)((*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 1) & MASK_S0) >> 16) * std::pow(2,-12);
				r = (float) (*(kpt_ptr + (blockoffset + i)*DWORDS_KPT + 1) & MASK_S1);
			}
			uint32_t id = nfeatures;
			
			// Exit condition for the block, i.e. not all 100 allowed features were filled
			if (x == 0 ||  y == 0){
				break;
			}
			
			if(x >= img_w - BORDER_EDGE || y >= img_h - BORDER_EDGE) {
				continue;
			}

			myFile << x << ", " << y << std::endl;

			vToDistributeKeys.push_back(cv::KeyPoint((float)x,(float)y,7.,a,r,0,id));
			nfeatures++;
		}
	}
	myFile.close();
	
	std::cout << "KeyPoints found: " << nfeatures << std::endl;

	for(int i = 0; i < vToDistributeKeys.size(); i++) {
		cv::KeyPoint kp = vToDistributeKeys[i];
		img_gray.at<uint8_t>(kp.pt.y, kp.pt.x) = 255;
	}
	cv::imwrite(argv[3], img_gray);
	std::cout << "Image written\n";

	// Cleanup
	free(kpt_ptr);
	kpt_ptr = NULL;

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
