#include <iostream>
#include <cstdio>
#include <ctime>
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
#include <opencv2/stitching/detail/warpers.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::detail;

const int numImage = 3;
const int ERROR = -1;

vector<Mat> homographies(numImage);

void homography() {
  for (int i = 1; i < numImage; i++) {
  // for (int i = numImage-1; i > 0; i--) {
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

void warpAndBlend(vector<Mat> images) {
  Mat stitched(images[0].rows, images[0].cols + images[1].cols, images[0].type());
  MultiBandBlender blender(2, true);

  //stores top left corners coordinates of each image
  vector<Mat> imagesWarped(numImage);
  vector<UMat> masks(numImage);

  blender.prepare(Rect(0, 0, images[0].cols * numImage, images[0].rows));
  std::clock_t start;
  double duration;
  start = std::clock();
  // the mask of an image is a white rectangle of the same size as the image
  for (int i = 0; i < numImage; i++) {
    masks[i].create(images[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));

    if (i != 0) {
      Mat previousImage = imagesWarped[i-1];
      Mat currentImage = images[i];

      for (int k = i-1; k >= 0; k--) {
        Mat h = homographies[k];
        // warpPerspective(currentImage, imagesWarped[i], h, Size(currentImage.cols * numImage, currentImage.rows));
        // warpPerspective(masks[i], masks[i], h, Size(currentImage.cols * numImage, currentImage.rows));
        warpAffine(currentImage, imagesWarped[i], h, Size(currentImage.cols * numImage, currentImage.rows));
        warpAffine(masks[i], masks[i], h, Size(currentImage.cols * numImage, currentImage.rows));
        currentImage = imagesWarped[i];
      }
    } else {
      imagesWarped[i] = images[i];
    }

    Mat imageS;
    imagesWarped[i].convertTo(imageS, CV_16S);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Warping Time " << to_string(i)  << " " << duration <<'\n';

    blender.feed(imageS, masks[i], Point(0, 0));
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Feeding Time " << to_string(i)  << " " << duration <<'\n';

  }

  Mat result, result_s, result_mask;
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"Aligning Time: "<< duration <<'\n';

  blender.blend(result_s, result_mask);
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"Blending Time: "<< duration <<'\n';

  result_s.convertTo(result, CV_8U);
  namedWindow("warped", WINDOW_NORMAL);
  resizeWindow("warped", 1024, 600);
  imshow ("warped", result);
  waitKey();
}

int main(int argc, char** argv) {
  // homography();
  // vector<Mat> images;
  //
  // for (int i = numImage; i > 0; i--) {
  //   string filename = "test" + to_string(i) + ".png";
  //   Mat im = imread(filename);
  //
  //   if (im.empty()) {
  //     cout << "Image " + filename + "is not found!" << endl;
  //     return ERROR;
  //   }
  //   images.push_back(im);
  // }
  //
  // warpAndBlend(images);

  return 0;
}
