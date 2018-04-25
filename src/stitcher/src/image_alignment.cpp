// Standard C++ I/O library.
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>

// OpenCV feature library.
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
  // Mat input = imread("images/Panorama.jpg");
  // Mat gray;
  // cvtColor(input, gray, CV_BGR2GRAY);
  //
  // //extract black background
  // Mat mask = gray > 0;
  // //extract outer contour
  // vector<vector<Point> > contours;
  // vector<Vec4i> hierarchy;
  //
  // findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0))
}
