#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/stitching/detail/warpers.hpp>

using namespace std;
using namespace cv;
using namespace cv::detail;

const int numImage = 1;
const int ERROR = -1;

class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
  Mat projection;
};

std::vector<CalibrationDetails> calibrations;

void getCalibrationDetails(int num_cam) {
  for (int i = 1; i < num_cam+1; i++) {
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
  getCalibrationDetails(numImage);
  vector<Mat> images;
  vector<Mat> imagesWarped(numImage);
  vector<Mat> imagesWarped_F(numImage);

  for (int i = numImage; i > 0; i--) {
    string filename = "test" + to_string(i) + ".png";
    Mat im = imread(filename, IMREAD_GRAYSCALE);

    if (im.empty()) {
      cout << "Image " + filename + "is not found!" << endl;
      return ERROR;
    }
    images.push_back(im);
  }

  Ptr<WarperCreator> warperCreator = makePtr<cv::CylindricalWarper>();
  Ptr<RotationWarper> warper = warperCreator->create(static_cast<float>(1000));

  for (int i = 0; i < numImage; i++) {
    Mat K, R;
    calibrations[i].camera_matrix.convertTo(K, CV_32F);
    calibrations[i].rectification.convertTo(R, CV_32F);

    warper->warp(images[i], K, R, INTER_LINEAR, BORDER_REFLECT, imagesWarped[i]);

    // imagesWarped[i].convertTo(imagesWarped_F[i], CV_32F);
    imshow("warped", imagesWarped[i]);
    waitKey();

  }
  return 0;
}
