#include <iostream>
#include <cstring>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>

extern "C" {
	#include "reconos.h"
	#include "reconos_app.h"
	#include "timer.h"
}

#define BASETYPE uint64_t
#define BYTES 8
#define MASK 7
#define DWORDS_KPT 7
#define DONEFLAG 0xffffffffffffffff
	
// Only used by R64
#define MASK_S0 0xffff000000000000
#define MASK_S1 0x0000ffff00000000
#define MASK_S2 0x00000000ffff0000
#define MASK_S3 0x000000000000ffff

#define MASK_W0 0xffffffff00000000
#define MASK_W1 0x00000000ffffffff

#define MAX_W 640 // In pixel
#define BYTEPERPIXEL 3
#define CC_W (MAX_W*BYTEPERPIXEL) // In Byte
#define CC_H 480
const BASETYPE BYTEMASK = 0xff;

#define BORDER_EDGE 16
#define WINDOW_SIZE 32
#define MAXPERBLOCK 100

#define NROWS (int)((CC_H - 2*BORDER_EDGE) / WINDOW_SIZE)
#define NCOLS (int)((MAX_W - 2*BORDER_EDGE) / WINDOW_SIZE)

typedef struct {
	uint64_t dw0, dw1, dw2, dw3;
} descriptor_t;


void print_help() {
	std::cout <<
		"Usage: fastdemo <sw_or_hw> <img_in> <depth_in> <img_out>\n"
		"For sw: fastdemo 0 <img_in> <depth_in> <img_out>\n"
		"For hw: fastdemo 1 <img_in> <depth_in> <img_out>"
	<< std::endl;
}

int main(int argc, char** argv) {
	if(argc != 5) {
		print_help();
		return 0;
	}
	
	reconos_init();
	reconos_app_init();
	timer_init();
	int clk = reconos_clock_threads_set(100000);

	int sw_or_hw = atoi(argv[1]);
	if(sw_or_hw == 0) { 
		std::cout << "Creating SW Thread" << std::endl;
		reconos_thread_create_swt_fast();
	}
	else if(sw_or_hw == 1) { 
		std::cout << "Creating HW Thread" << std::endl;
		reconos_thread_create_hwt_fast();
	}
	else {
		std::cout << "Invalid mode" << std::endl;
		print_help();
	}

	BASETYPE blocks = NROWS * NCOLS;
	BASETYPE mem_size_kpt = MAXPERBLOCK * blocks * DWORDS_KPT;

	cv::Mat img_i = cv::imread(argv[2]);
	cv::Mat img_d = cv::imread(argv[3], cv::IMREAD_ANYDEPTH);
	BASETYPE img_w = img_i.cols;
	BASETYPE _img_w = img_w * BYTEPERPIXEL;
	BASETYPE img_h = img_i.rows;
	
	//cv::Mat _dummy = cv::Mat::zeros(2,img_w,img_i.type());
	//img_i.push_back(_dummy);
	uint8_t* ptr_i = (uint8_t*)img_i.data;
	uint16_t* ptr_d = (uint16_t*)img_d.data;
	BASETYPE* kpt_ptr = (BASETYPE*)malloc(mem_size_kpt * sizeof(BASETYPE));
	memset(kpt_ptr, 0, mem_size_kpt * sizeof(BASETYPE));

	std::vector<cv::KeyPoint> vToDistributeKeys;
	vToDistributeKeys.reserve(MAXPERBLOCK*NROWS*NCOLS);
	
	std::cout << "Starting thread work" << std::endl;
	unsigned int t_start, t_end;
	
	t_start = timer_get();
	{
		// Thread computations
		mbox_put(rcsfast_sw2rt, (BASETYPE)ptr_i);
		mbox_put(rcsfast_sw2rt, (BASETYPE)ptr_d);
		mbox_put(rcsfast_sw2rt, (BASETYPE)kpt_ptr);
		mbox_put(rcsfast_sw2rt, (BASETYPE)img_w);
		mbox_put(rcsfast_sw2rt, (BASETYPE)_img_w);
		mbox_put(rcsfast_sw2rt, (BASETYPE)img_h);
		mbox_put(rcsfast_sw2rt, (BASETYPE)200000);
		std::cout << "Waiting for answer" << std::endl;
	
		BASETYPE ret;
		do {
			ret = mbox_get(rcsfast_rt2sw);
			std::cout << ret << std::endl;
		}
		while(ret != DONEFLAG);
	}
	t_end = timer_get();
	std::cout << "Thread work complete\n";

	std::ofstream myFile;
	if(sw_or_hw == 0)
		myFile.open("res_sw.txt", std::ios_base::trunc);
	else
		myFile.open("res_hw.txt", std::ios_base::trunc);
	myFile << "x, y, xU, yU, xR, depth, Angle, Response, Desc0, Desc1, Desc2, Desc3\n";

	std::vector<descriptor_t> unfltd_desc;
	// Construct result vector from memory
	uint32_t nfeatures = 0;
	for(size_t b = 0; b < blocks; b++){
		uint32_t blockoffset = b * MAXPERBLOCK;
		BASETYPE inBlock = (BASETYPE)*(kpt_ptr + (blockoffset + 0)*DWORDS_KPT);
		std::cout << "Row " << (int)(b/NCOLS) << " Col " << b%NCOLS << ": " << inBlock << std::endl;
		for(size_t i = 0; i < inBlock; i++){
			uint16_t x, y, depth, r;
			float angle, xU, yU, xR;

			uint64_t dword0 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 0)*DWORDS_KPT);
			x = ((dword0 & MASK_S0) >> 48);
			y = ((dword0 & MASK_S1) >> 32);
			depth = ((dword0 & MASK_S2) >> 16);
			r = (dword0 & MASK_S3);
			
			uint64_t dword1 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 1)*DWORDS_KPT);
			xU = ((dword1 & MASK_W0) >> 32) * std::pow(2,-8);
			yU = (dword1 & MASK_W1) * std::pow(2,-8);

			uint64_t dword2 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 2)*DWORDS_KPT);
			xR = ((dword2 & MASK_W0) >> 32) * std::pow(2,-8);
			angle = (dword2 & MASK_W1) * std::pow(2,-12);
			
			uint64_t dword3 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 3)*DWORDS_KPT);
			uint64_t dword4 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 4)*DWORDS_KPT);
			uint64_t dword5 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 5)*DWORDS_KPT);
			uint64_t dword6 = *(kpt_ptr + (blockoffset + (i*DWORDS_KPT) + 6)*DWORDS_KPT);

			uint32_t id = nfeatures;
			if(x >= img_w - BORDER_EDGE || y >= img_h - BORDER_EDGE) {
				continue;
			}
			// Exit condition for the block, i.e. not all 100 allowed features were filled
		//	if (x == 0 ||  y == 0){
		//		break;
		//	}

			myFile << x << ", " << y << ", " << xU << ", " << yU << ", " << xR << ", " << depth << ", " << angle << ", " << r << ", " << dword3 << ", " << dword4 << ", " << dword5 << ", " << dword6 << std::endl;

			vToDistributeKeys.push_back(cv::KeyPoint((float)x,(float)y,7.,angle,r,0,id));
			descriptor_t tmp = {dword3,dword4,dword5,dword6};
			unfltd_desc.push_back(tmp);
			nfeatures++;
		}
	}
	myFile.close();
	
	std::cout << "Thread compute took " << timer_toms(t_end - t_start) << " ms" << std::endl;
	std::cout << "KeyPoints found: " << nfeatures << std::endl;
	
	cv::Mat mToDistributeDescriptors = cv::Mat((int)vToDistributeKeys.size(), 32, CV_8U);

	for(size_t i = 0; i < vToDistributeKeys.size(); i++) {
		cv::KeyPoint kp = vToDistributeKeys[i];
		img_i.at<cv::Vec3b>(kp.pt.y, kp.pt.x) = cv::Vec3b(255,255,255);
	}
	cv::imwrite(argv[4], img_i);
	std::cout << "Image written\n\n";

	// Cleanup
	free(kpt_ptr);
	kpt_ptr = NULL;

	reconos_app_cleanup();
	reconos_cleanup();
	
	return 0;
}
