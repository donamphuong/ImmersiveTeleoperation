#include <iostream>
#include <cstdio>
#include <ctime>
#include <opencv2/core/ocl.hpp>
// #include "opencv2/opencv_modules.hpp"
#include <stdlib.h>
#include <string>
#include <stdio.h>
// #include <ros/ros.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "details.cpp"
#include <tbb/tbb.h>

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

void sum(int a, int b, int &sum) {
  sum = a+ b;
}

int main(int argc, char** argv) {
  int sum1 = 0, sum2 = 0;
  tbb::parallel_invoke(
    [&]{sum1 = 1 + 2;},
    [&]{sum2 = 3 + 4;}
  );

cout << sum1 + sum2 << endl;

}
