
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

std::string to_string(int x) {
  std::stringstream stream;
  stream << x;
  return stream.str();
}

// main().
int main(int argv, char ** argc)
{
    int numImages = 2;
    std::vector<Mat> images;

    for (int i = 1; i < numImages+1; i++) {
      std::cout << ("images/ex" + to_string(i) + ".png") << std::endl;
      Mat im = imread("images/ex" + to_string(i) + ".png");
      images.push_back(im);
    }

    Mat pano;
    Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA, true);
    Stitcher::Status status = stitcher->stitch(images, pano);

    if (status != Stitcher::OK) {
      std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
      return 1;
    }

    cv::imshow("Transformed image", pano);

    cvWaitKey(0);

    return 0;
}
