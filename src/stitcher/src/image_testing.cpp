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

using namespace std;
using namespace cv;
// using namespace cv::xfeatures2d;

class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
  Mat projection;
};

std::vector<CalibrationDetails> calibrations;

void getCalibrationDetails() {
  for (int i = 1; i < 2+1; i++) {
    CalibrationDetails cal;
    std::string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/calibration/camera" + to_string(i) + ".yaml";
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
      std::cout << "Could not open the configuration file" << std::endl;
      exit(-1) ;
    }

    fs["camera_matrix"] >> cal.camera_matrix;
    fs["distortion_coefficients"] >> cal.distortion;
    fs["rectification_matrix"] >> cal.rectification;
    fs["projection_matrix"] >> cal.projection;
    fs.release();

    calibrations.push_back(cal);
  }
}


int main(int argc, char** argv) {
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
