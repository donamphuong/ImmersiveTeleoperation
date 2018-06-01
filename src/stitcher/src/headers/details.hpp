#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define startCamera 4
#define numImage 3
#define ERROR -1
#define WEIGHT_EPS 1e-5f

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
