#ifndef DETAILS_HPP_
#define DETAILS_HPP_

#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define startCamera 1
#define numImage 6
#define ERROR -1

class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
};

std::vector<CalibrationDetails> calibrations;

void printVector(vector<Mat> vect);
void getCalibrationDetails();
vector<Mat> homography();

#endif
