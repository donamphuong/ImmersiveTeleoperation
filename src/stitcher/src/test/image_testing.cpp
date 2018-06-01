#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../details.cpp"
#include <tbb/tbb.h>
#include "test.h"

using namespace std;
using namespace cv;
using namespace tbb;

void turnOnVideos() {
  getCalibrationDetails();
  VideoCapture cap1;
  cap1.open(1);
  VideoCapture cap2;
  cap2.open(2);

  cap1.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  cap1.set(CAP_PROP_FRAME_WIDTH,1920);
  cap1.set(CAP_PROP_FRAME_HEIGHT,1080);
  cap1.set(CAP_PROP_AUTOFOCUS,true);

  cap2.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  cap2.set(CAP_PROP_FRAME_WIDTH,1920);
  cap2.set(CAP_PROP_FRAME_HEIGHT,1080);
  cap2.set(CAP_PROP_AUTOFOCUS,true);

  namedWindow("frame1", WINDOW_NORMAL);
  resizeWindow("frame1", 1024, 600);
  namedWindow("frame2", WINDOW_NORMAL);
  resizeWindow("frame2", 1024, 600);

  while (true) {
    Mat frame1;
    cap1 >> frame1;
    Mat frame2;
    cap2 >> frame2;

    Mat corrected1, corrected2;

    undistort(frame1, corrected1, calibrations[0].camera_matrix, calibrations[0].distortion);
    imshow("frame1", frame1);
    cv::waitKey(1);
    undistort(frame2, corrected2, calibrations[1].camera_matrix, calibrations[1].distortion);
    imshow("frame2", frame2);
    cv::waitKey(1);

  }
}

// void testCPU(pixel *src, 
//   //             pixel *dst, 
//   //             const int pixelGroupSizeX, 
//   //             const int pixelGroupSizeY) {
//   //  // images area in the final stitched image
//   // Mat imLeftArea = aligned(Rect(0, 0, im1.cols, im1.rows));
//   // Mat imRightArea = aligned(Rect(im1.cols, 0, im2.cols, im2.rows));

//   // Mat roiImgLeft = im1(Rect(0, 0, im1.cols, im2.rows));
//   // Mat roiImgRight = im2Warped(Rect(im1.cols, 0, im2.cols, im2.rows));

//   // roiImgLeft.copyTo(imLeftArea);
//   // roiImgRight.copyTo(imRightArea);

//   // for (int x = 0; x < pixelGroupSizeX; x++) {
//   //   for (int y = 0; y < pixelGroupSizeY; y++) {
//   //     if
//   //   }
//   // }

// }

void processUsingOpenCvCpu() {
	//Read input image from the disk
	Mat input = imread("test34.png");
	if(input.empty())
	{
	        std::cout<<"Image Not Found: "<< "test34.png" << std::endl;
		// return;
	}
 
	//Create output image
	Mat output;
 
	clock_t start = clock();

	resize(input, output, Size(), .25, 0.25, INTER_AREA); // downscale 4x on both x and y
 
	double duration = (clock() - start) / (double) CLOCKS_PER_SEC;
	cout << "OpenCv Cpu code ran in:" << duration << " secs" << endl;
 }

 void processUsingGpu() {
   //Read input image from the disk
	cv::Mat input = cv::imread("test34.png");
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< "test34.png" << std::endl;
		// return;
	}
 
	//Create output image
	Size newSize( input.size().width / 4, input.size().height / 4 );
	Mat output (newSize, input.type());
 
	downscaleCuda(input, output);
 }

int main(int argc, char** argv) {
  clock_t start = clock();
  processUsingOpenCvCpu();
  cout << "Total time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
  return 0;
 }
