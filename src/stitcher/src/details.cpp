#include "headers/details.hpp"
  int NUM_IMAGES_PANORAMA = 6;

void printVector(vector<Mat> vect) {
  for (int i = 0; i < vect.size(); i++) {
    cout << vect[i] << endl;
  }
}

void getCalibrationDetails() {
  for (int i = 1; i <= NUM_IMAGES_PANORAMA; i++) {
    CalibrationDetails cal;
    std::string filename = "./src/stitcher/calibration/camera" + to_string(i) + ".yaml";
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
      std::cout << "Could not open the configuration file" << std::endl;
      exit(-1) ;
    }

    fs["camera_matrix"] >> cal.camera_matrix;
    fs["distortion_coefficients"] >> cal.distortion;
    // fs["rectification_matrix"] >> cal.rectification;
    fs.release();

    calibrations.push_back(cal);
  }

  // Get existing homographies to find new camera matrix for all six cameras
  vector<Mat> homographies = homography();
  calibrations[startCamera-1].rectification = Mat_<double>::eye(3, 3);

  for (int i = startCamera; i < NUM_IMAGES_PANORAMA; i++) {
    vector<Mat> rotations, translations, normals;
    decomposeHomographyMat(homographies[i-1], calibrations[i].camera_matrix, rotations, translations, normals);
    calibrations[i].rectification = calibrations[i-1].rectification * rotations[0];
  }
}

vector<Mat> homography() {
  vector<Mat> homographies(NUM_IMAGES_PANORAMA-1);
  // for (int i = 1; i < numImage; i++) {
  for (int i = 1; i < NUM_IMAGES_PANORAMA; i++) {
    Mat h;
    string filename = "./src/stitcher/homography/H" +
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
