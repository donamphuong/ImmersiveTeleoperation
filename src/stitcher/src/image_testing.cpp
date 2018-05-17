#include <iostream>
#include <cstdio>
#include <ctime>
// #include "opencv2/opencv_modules.hpp"
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "details.cpp"

using namespace std;
using namespace cv;

void turnOnVideos() {
  // getCalibrationDetails();
  // VideoCapture cap1;
  // cap1.open(1);
  // VideoCapture cap2;
  // cap2.open(2);
  //
  // cap1.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  // cap1.set(CAP_PROP_FRAME_WIDTH,1920);
  // cap1.set(CAP_PROP_FRAME_HEIGHT,1080);
  // cap1.set(CAP_PROP_AUTOFOCUS,true);
  //
  // cap2.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  // cap2.set(CAP_PROP_FRAME_WIDTH,1920);
  // cap2.set(CAP_PROP_FRAME_HEIGHT,1080);
  // cap2.set(CAP_PROP_AUTOFOCUS,true);
  //
  // while (true) {
  //   Mat frame1;
  //   cap1 >> frame1;
  //   Mat frame2;
  //   cap2 >> frame2;
  //
  //   Mat corrected1, corrected2;
  //
  //   undistort(frame1, corrected1, calibrations[0].camera_matrix, calibrations[0].distortion);
  //   imshow("frame1", frame1);
  //   cv::waitKey(1);
  //   undistort(frame2, corrected2, calibrations[1].camera_matrix, calibrations[1].distortion);
  //   imshow("frame2", frame2);
  //   cv::waitKey(1);
  //
  // }
}

// MatND calcHist(Mat im_gray) {
//   int histSize = 256;
//   // for 8 bit image
//   float range[] = { 0, 255 };
//   const float *ranges[] = { range };
//
//   //calculate histogram
//   MatND hist;
//   calcHist(&im_gray, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false);
//   normalize
//   return hist;
// }

vector<float> calcHist(Mat im_gray) {
  vector<float> hist(256, 0);
  for (int r = 0; r < im_gray.rows; r++) {
    for (int c = 0; c < im_gray.cols; c++ ) {
      hist[(int) im_gray.at<uchar>(r, c)]++;
    }
  }
  return hist;
}

vector<float> calcCumHist(vector<float> hist) {
  vector<float> cumHist = hist;

  for (int i = 1; i < 256; i++) {
    cumHist[i] += cumHist[i-1];
  }

  return cumHist;
}

// aim to transform an input image to an output image fitting a specific histogram
Mat histMatch(Mat image, Mat image_ref) {
  vector<float> hist = calcHist(image);
  vector<float> hist_ref = calcHist(image_ref);

  vector<float> cumHist = calcCumHist(hist);
  vector<float> cumHist_ref = calcCumHist(hist_ref);

  int pixels = image.cols * image.rows;
  int pixels_ref = image_ref.cols * image_ref.rows;
  int K = 256;
  //normalize pixel intensity
  for (int i = 0; i < K; i++) {
    cumHist[i] = cumHist[i]/pixels;
    cumHist_ref[i] = cumHist_ref[i]/pixels_ref;
  }


  int new_values[256];
  //a is the current pixel intensity
  for (int a = 0; a < K; a++) {
    // j is the new pixel intensity
    for (int j = K-1; j >= 0; j--) {
      new_values[a] = j;
      if (cumHist[a] > cumHist_ref[j]) {
        break;
      }
    }
  }

  for (int i = 0; i < image.rows; i++) {
    for (int j = 0; j < image.cols; j++) {
      int pixel_intensity = image.at<uchar>(i, j);
      int new_intensity = new_values[pixel_intensity];
      image.at<uchar>(i, j) = new_intensity;
    }
  }
  Mat color_image_ref, color_image;
  cvtColor(image_ref, color_image_ref, COLOR_GRAY2BGR);
  cvtColor(image, color_image, COLOR_GRAY2BGR);
  imshow("image_ref", color_image_ref);
  imshow("image_hm", color_image);
  waitKey();
  return image;
}

void colour_mapping(Mat test_overlap, Mat test_overlap_hm) {
  map<int, vector<Point>> eps;

  for (int i = 0; i < test_overlap.rows; i++) {
    for (int j = 0; j < test_overlap.cols; j++) {
      int intensity = test_overlap.at<uchar>(i, j);
      vector<Point> vect;

      if (eps.find(intensity) != eps.end()) {
        vect = eps[intensity];
      }

      vect.push_back(Point(i, j));
      eps[intensity] = vect;
    }
  }

  int map[3][256];
  //TODO:change into dynamic programming
  for (int k = 0; k < 256; k++) {
    int minRed = INT_MAX;
    int minGreen = INT_MAX;
    int minBlue = INT_MAX;
    vector<Point> positions = eps[k];

    if (positions.empty()) {

    } else {
      //all possible values for c
      for (int j = 0; j < 256; j++) {
        int sumRed, sumGreen, sumBlue;
        for (int i = 0; i < positions.size(); i++) {
          Vec3b bgrPixel = test_overlap_hm.at<Vec3b>(positions[i]);
          sumRed += pow(bgrPixel.val[0] - i, 2);
          sumGreen += pow(bgrPixel.val[1] - i, 2);
          sumBlue += pow(bgrPixel.val[2] - i, 2);
        }

        minGreen = min(minGreen, sumGreen);
        minRed = min(minRed, sumRed);
        minBlue = min(minBlue, sumBlue);
      }

      map[0][k] = floor(minRed + 0.5);
      map[1][k] = floor(minGreen + 0.5);
      map[2][k] = floor(minBlue + 0.5);
    }
  }

}

// void warpAndBlend(vector<Mat> images) {
//   Mat aligned(images[0].rows, images.size() * images[0].cols, images[0].type());
//   //stores top left corners coordinates of each image
//   vector<Mat> imagesWarped(images.size());
//   vector<Mat> masks(images.size());
//   vector<Mat> masksWarped(images.size());
//
//   std::clock_t start;
//   double duration;
//   start = std::clock();
//   // the mask of an image is a white rectangle of the same size as the image
//   for (int i = 0; i < images.size(); i++) {
//     masks[i].create(images[i].size(), CV_8U);
//     masks[i].setTo(Scalar::all(255));
//
//     Rect area;
//     if (i != 0) {
//       Mat previousImage = imagesWarped[i-1];
//       Mat currentImage = images[i];
//
//       Mat h = homographies[i-1];
//       warpAffine(currentImage, imagesWarped[i], h, Size(currentImage.cols * numImage, currentImage.rows));
//       warpAffine(masks[i], masksWarped[i], h, Size(currentImage.cols * numImage, currentImage.rows));
//
//       area = Rect(images[0].cols * i, 0, images[0].cols, images[0].rows);
//
//     } else {
//       imagesWarped[i] = images[i];
//       area = Rect(0, 0, images[0].cols, images[0].rows);
//     }
//
//     imagesWarped[i](area).copyTo(aligned(area));
//   }
//
//   duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
//   std::cout<<"printf: "<< duration <<'\n';
//
//   // namedWindow("warped", WINDOW_NORMAL);
//   // resizeWindow("warped", 1024, 600);
//   // imshow ("warped", aligned);
//   // waitKey();
// }

int main(int argc, char** argv) {
  // affine();
  //
  // vector<Mat> images;
  // // images.push_back(imread("test3.png"));
  // images.push_back(imread("test2.png"));
  // images.push_back(imread("test1.png"));
  // histCalc(images);

  Mat im1 = imread("test1 (copy).png"), im_gray1;
  Mat im2 = imread("test2 (copy).png"), im_gray2;
  cvtColor(im1, im_gray1, COLOR_BGR2GRAY);
  cvtColor(im2, im_gray2, COLOR_BGR2GRAY);

  clock_t start = clock();
  double duration;

  Mat t_o_hm = histMatch(im_gray1, im_gray2);

  duration = (clock() - start) / (double) CLOCKS_PER_SEC;
  cout << duration << endl;

}
