#include <iostream>
#include <cstdio>
#include <ctime>
// #include "opencv2/opencv_modules.hpp"
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

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;
class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
  Mat projection;
};

/* 0.005003 for feature detection using chessboard pattern and for the program
to deduce a homography*/
std::vector<CalibrationDetails> calibrations;

int main(int argc, char** argv) {
  cv::Mat im_ref, im_cmp;

  resize(cv::imread("images/test1.jpg", IMREAD_GRAYSCALE), im_ref, Size(800, 600));
  resize(cv::imread("images/test2.jpg", IMREAD_GRAYSCALE), im_cmp, Size(800, 600));
  Size patternSize = Size(8, 6);
  std::clock_t start;
  double duration;
  start = std::clock();


  //! [find-corners]
  vector<Point2f> corners1, corners2;
  bool found1 = findChessboardCorners(im_ref, patternSize, corners1);
  bool found2 = findChessboardCorners(im_cmp, patternSize, corners2);
  // ! [find-corners]

  // Find homography
  Mat h = findHomography( Mat(corners1), Mat(corners2), RANSAC);
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"printf: "<< duration <<'\n';
  Mat im_stitch;
  // Use homography to warp image
  warpPerspective(im_ref, im_stitch, h, Size(im_ref.cols * 3, im_ref.rows));
  Mat half = im_stitch(Rect(0, 0, im_cmp.cols, im_cmp.rows));
  im_cmp.copyTo(half);
  imshow("aligned", im_stitch);

  waitKey(0);

  return 0;

}
