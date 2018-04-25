#include <stdio.h>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

void calibrate(int cam, Mat img, Mat newImg) {
  Mat camera_matrix, distortion, rectification, projection;

  std::string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/calibration/head_camera.yaml";
  FileStorage fs(filename, FileStorage::READ);

  if (!fs.isOpened()) {
    std::cout << "Could not open the configuration file" << std::endl;
    return ;
  }

  fs["camera_matrix"] >> camera_matrix;
  fs["distortion"] >> distortion;
  fs["rectification"] >> rectification;
  fs["projection"] >> projection;
  fs.release();

  undistort(img, newImg, camera_matrix, distortion);
}
