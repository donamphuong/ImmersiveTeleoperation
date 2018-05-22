#include <iostream>
#include <cstdio>
#include <ctime>
// #include "opencv2/opencv_modules.hpp"
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "details.cpp"

using namespace std;
using namespace cv;

void turnOnVideos() {
  // getCalibrationDetails();
  // VideoCapture cap1;
  // cap1.open(1);
  // VideoCapture cap2;
  // cap2.open(2);
  //
  // cap1.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  // cap1.set(CAP_PROP_FRAME_WIDTH,1920);
  // cap1.set(CAP_PROP_FRAME_HEIGHT,1080);
  // cap1.set(CAP_PROP_AUTOFOCUS,true);
  //
  // cap2.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  // cap2.set(CAP_PROP_FRAME_WIDTH,1920);
  // cap2.set(CAP_PROP_FRAME_HEIGHT,1080);
  // cap2.set(CAP_PROP_AUTOFOCUS,true);
  //
  // while (true) {
  //   Mat frame1;
  //   cap1 >> frame1;
  //   Mat frame2;
  //   cap2 >> frame2;
  //
  //   Mat corrected1, corrected2;
  //
  //   undistort(frame1, corrected1, calibrations[0].camera_matrix, calibrations[0].distortion);
  //   imshow("frame1", frame1);
  //   cv::waitKey(1);
  //   undistort(frame2, corrected2, calibrations[1].camera_matrix, calibrations[1].distortion);
  //   imshow("frame2", frame2);
  //   cv::waitKey(1);
  //
  // }
}

// MatND calcHist(Mat im_gray) {
//   int histSize = 256;
//   // for 8 bit image
//   float range[] = { 0, 255 };
//   const float *ranges[] = { range };
//
//   //calculate histogram
//   MatND hist;
//   calcHist(&im_gray, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false);
//   normalize
//   return hist;
// }

int main(int argc, char** argv) {
  numImage = 1;
  VideoCapture cap[numImage];
// cout<<cv::getBuildInformation()<<endl;
  for (int i = 0; i < numImage; i++) {
    // string gst = "v4l2src device=/dev/video0 ! video/x-raw, format=BGR, width=640, height=480, framerate=120/1 ! videoconvert ! appsink";
    cap[i].open(i);

    if (!cap[i].isOpened()) {
      cout << "Cannot open device " << i << endl;
      exit(ERROR);
    }

    cap[i].set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    cap[i].set(CAP_PROP_FRAME_WIDTH,1920);
    cap[i].set(CAP_PROP_FRAME_HEIGHT,1080);
    cap[i].set(CAP_PROP_AUTOFOCUS,true);
    cap[i].set(CAP_PROP_FPS,30);
  }



  Mat frame;
  while (true) {

    cap[0].read(frame);
    imshow("view", frame);
    if (waitKey(30) == 27) {
      return ERROR;
    }
  }
}
