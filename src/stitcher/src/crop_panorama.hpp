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

bool checkInteriorExterior(const cv::Mat&mask, const cv::Rect&interiorBB, int&top, int&bottom, int&left, int&right);
Rect cropPanorama();
