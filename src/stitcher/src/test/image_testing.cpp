#include <iostream>
#include <cstdio>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <vector>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "../headers/details.hpp"
#include "../headers/stitch.hpp"
#include "../headers/precomp.hpp"
#include <tbb/tbb.h>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

using namespace std;
using namespace cv;
using namespace tbb;


void turnOnVideos() {
  getCalibrationDetails();
  VideoCapture cap1;
  cap1.open(startCamera-1);
  VideoCapture cap2;
  cap2.open(startCamera);

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
    undistort(frame1, corrected1, calibrations[startCamera-1].camera_matrix, calibrations[startCamera-1].distortion, calibrations[0].camera_matrix);
    cout << "undistort time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
    imshow("frame1", frame1);
    cv::waitKey(1);
    undistort(frame2, corrected2, calibrations[startCamera].camera_matrix, calibrations[startCamera].distortion, calibrations[0].camera_matrix);
    imshow("frame2", frame2);
    cv::waitKey(1);

  }
}

void testStitch() {
  getCalibrationDetails();
  precomp();
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

  while (true) {
    vector<Mat> images;
    Mat frame1;
    cap1 >> frame1;
    Mat frame2;
    cap2 >> frame2;

    Mat corrected1, corrected2;

    clock_t start = clock();
    undistort(frame1, corrected1, calibrations[startCamera-1].camera_matrix, calibrations[startCamera-1].distortion, calibrations[0].camera_matrix);
    cout << "undistort time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
    images.push_back(frame1);
    undistort(frame2, corrected2, calibrations[startCamera].camera_matrix, calibrations[startCamera].distortion, calibrations[0].camera_matrix);
    images.push_back(frame2);

    Mat result;
    stitch(images, result);
    imshow("result", result);
    waitKey(1);

  }
}


int main(int argc, char** argv) {
  turnOnVideos();
  // testStitch();
}
