#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int numImage = 3;
const int ERROR = -1;

class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
  Mat projection;
};

std::vector<CalibrationDetails> calibrations;
vector<Mat> homographies(numImage);

void getCalibrationDetails(int num_cam) {
  for (int i = num_cam; i > 0; i--) {
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

void affine() {
  // for (int i = 1; i < numImage; i++) {
  for (int i = numImage-1; i > 0; i--) {
    Mat h;
    string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/affine/A" +
                      to_string(i) + to_string(i+1) + ".yaml";
    FileStorage file(filename, FileStorage::READ);

    if (!file.isOpened()) {
      cout << "File " + filename + " cannot be opened!";
      exit(ERROR);
    }

    file["affine"] >> h;
    file.release();
    homographies[i-1] = h;
  }
}

void homography() {
  // for (int i = 1; i < numImage; i++) {
  for (int i = 1; i < numImage; i++) {
    Mat h;
    string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/homography/H" +
                      to_string(i+1) + to_string(i) + ".yaml";
    FileStorage file(filename, FileStorage::READ);

    if (!file.isOpened()) {
      cout << "File " + filename + " cannot be opened!";
      exit(ERROR);
    }

    file["homography"] >> h;
    file.release();
    homographies[i-1] = h;
  }
}
