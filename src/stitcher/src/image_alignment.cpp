#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "details.cpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

void warpImage(int index,
                Mat image,
                vector<Mat> &imagesWarped,
                Ptr<RotationWarper> warper) {
  Mat K, R;
  calibrations[index].camera_matrix.convertTo(K, CV_32F);
  calibrations[index].rectification.convertTo(R, CV_32F);

  warper->warp(image, K, R, INTER_LINEAR, BORDER_REFLECT, imagesWarped[index]);
}

void stitch(vector<Mat> images) {
  Mat stitched(images[0].rows, images[0].cols + images[1].cols, images[0].type());

  //stores top left corners coordinates of each image
  vector<Mat> imagesWarped(numImage);
  vector<UMat> masks(numImage);
  vector<Point> corners(numImage, Point(0, 0));
  vector<Size> sizes(numImage, images[0].size());

  Ptr<WarperCreator> warperCreator = makePtr<cv::CylindricalWarper>();
  Ptr<RotationWarper> warper = warperCreator->create(static_cast<float>(1000));


  Ptr<Blender> blender = Blender::createDefault(Blender::MULTI_BAND, true);
  blender->prepare(Rect(0, 0, images[0].cols * numImage, images[0].rows));
  // Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN_BLOCKS);
  // compensator->feed(corners, imagesWarped, masks);

  std::clock_t start;
  double duration;
  start = std::clock();
  // the mask of an image is a white rectangle of the same size as the image
  for (int i = 0; i < numImage; i++) {
    masks[i].create(images[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));

    warpImage(i, images[i], imagesWarped, warper);

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

    // compensator->apply(i. corners[i], imagesWarped[i], masks[i]);

    blender->feed(imageS, masks[i], Point(0, 0));
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Feeding Time " << to_string(i)  << " " << duration <<'\n';

  }

  Mat result, result_s, result_mask;
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"Aligning Time: "<< duration <<'\n';

  blender->blend(result_s, result_mask);
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"Blending Time: "<< duration <<'\n';

  result_s.convertTo(result, CV_8U);
  namedWindow("warped", WINDOW_NORMAL);
  resizeWindow("warped", 1024, 600);
  imshow ("warped", result);
  waitKey();
}

int main(int argc, char** argv) {
  getCalibrationDetails(numImage);
  homography();
  vector<Mat> images;

  for (int i = numImage; i > 0; i--) {
    string filename = "test" + to_string(i) + ".png";
    Mat im = imread(filename);

    if (im.empty()) {
      cout << "Image " + filename + "is not found!" << endl;
      return ERROR;
    }
    images.push_back(im);
  }

  stitch(images);

  return 0;
}
