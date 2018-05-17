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
};

vector<Mat> rotationMatrix;
Mat newCameraMatrix;

void printVector(vector<Mat> vect);
void findRotationMatrix();
Mat findNewCameraMatrix();
void getCalibrationDetails();
vector<Mat> homography();

void printVector(vector<Mat> vect) {
  for (int i = 0; i < vect.size(); i++) {
    cout << vect[i] << endl;
  }
}

void getCalibrationDetails() {
  std::vector<CalibrationDetails> calibrations;

  for (int i = numImage; i > 0; i--) {
    CalibrationDetails cal;
    std::string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/calibration/camera" + to_string(i) + ".yaml";
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
      std::cout << "Could not open the configuration file" << std::endl;
      exit(-1) ;
    }

    fs["camera_matrix"] >> cal.camera_matrix;
    fs["distortion_coefficients"] >> cal.distortion;
    fs.release();

    // rotationMatrix.push_back(cal.rectification);
    calibrations.push_back(cal);
  }

  // Find new camera matrix
  Size image_size = Size(1920, 1080);

  // for (int i = 0; i < numImage; i++) {
  //   newCameraMatrix = getOptimalNewCameraMatrix(calibrations[i].camera_matrix, calibrations[i].distortion, image_size, 1);
  //   cout << newCameraMatrix << endl;
  // }
  newCameraMatrix = calibrations[0].camera_matrix;

  // Get existing homographies to find new camera matrix for all six cameras
  vector<Mat> homographies = homography();
  rotationMatrix.push_back(Mat_<double>::eye(3, 3));

  for (int i = 0; i < numImage-1; i++) {
    vector<Mat> rotations, translations, normals;
    decomposeHomographyMat(homographies[i], newCameraMatrix, rotations, translations, normals);
    rotationMatrix.push_back(rotationMatrix[i] * rotations[0]);
  }
}

// void affine() {
//   // for (int i = 1; i < numImage; i++) {
//   for (int i = numImage-1; i > 0; i--) {
//     Mat h;
//     string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/affine/A" +
//                       to_string(i) + to_string(i+1) + ".yaml";
//     FileStorage file(filename, FileStorage::READ);
//
//     if (!file.isOpened()) {
//       cout << "File " + filename + " cannot be opened!";
//       exit(ERROR);
//     }
//
//     file["affine"] >> h;
//     file.release();
//     homographies[i-1] = h;
//   }
// }

vector<Mat> homography() {
  vector<Mat> homographies(numImage);
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

  return homographies;
}
