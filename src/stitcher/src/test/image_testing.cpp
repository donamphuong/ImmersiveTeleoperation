#include <iostream>
#include <cstdio>
#include <ctime>
#include <cv_bridge/cv_bridge.h>

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
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

    clock_t start = clock();
    undistort(frame1, corrected1, calibrations[0].camera_matrix, calibrations[0].distortion);
    cout << "undistort time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
    imshow("frame1", frame1);
    cv::waitKey(1);
    undistort(frame2, corrected2, calibrations[1].camera_matrix, calibrations[1].distortion);
    imshow("frame2", frame2);
    cv::waitKey(1);

  }
}

// void testCPU() {
//   int N = 1<<20; // 1M elements

//   float *x = new float[N];
//   float *y = new float[N];

//   clock_t start = clock();
//   // initialize x and y arrays on the host
//   for (int i = 0; i < N; i++) {
//     x[i] = 1.0f;
//     y[i] = 2.0f;
//   }
//   cout << "time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;

//   // Run kernel on 1M elements on the CPU
//   for (int i = 0; i < N; i++)
//       y[i] = x[i] + y[i];

//   // Free memory
//   delete [] x;
//   delete [] y;
// }

vector<UMat> undistortMap1(numImage);
vector<UMat> undistortMap2(numImage);

const Size image_size = Size(1920, 1080);

void getUndistortMap() {
  Mat I = Mat_<double>::eye(3,3);

  for (int cam = 0; cam < numImage; cam++) {
    CalibrationDetails cal = calibrations[cam];
    initUndistortRectifyMap(cal.camera_matrix, cal.distortion, I, Mat(), image_size, undistortMap1[cam].type(), undistortMap1[cam], undistortMap2[cam]);
  }
}


int main(int argc, char** argv) {
  turnOnVideos();

 }
