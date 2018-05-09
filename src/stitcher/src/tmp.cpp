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
#include <opencv2/stitching/detail/warpers.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::detail;

const int numImage = 2;
const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;
class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
  Mat projection;
};

std::vector<CalibrationDetails> calibrations;

Mat homography(Mat im1, Mat im2) {
  // int minHessain = 400;
  // Mat gray1, gray2;
  // cvtColor(im1, gray1, COLOR_BGR2GRAY);
  // cvtColor(im2, gray2, COLOR_BGR2GRAY);
  //
  // // Copy image into GPU memory
  // cuda::GpuMat img1Gray(gray1);
  // cuda::GpuMat img2Gray(gray2);
  //
  // // Variables to store keypoints and descriptors
  // cuda::GpuMat keypoints1, keypoints2;
  // cuda::GpuMat descriptors1, descriptors2;
  //
  // // Detect ORB features and compute descriptors.
  // cuda::SURF_CUDA surf(minHessain);
  // surf(img1Gray, cuda::GpuMat(), keypoints1, descriptors1);
  // surf(img2Gray, cuda::GpuMat(), keypoints2, descriptors2);
  //
  // // Match features.
  // vector<vector<DMatch> > matches;
  // Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher();
  // matcher->knnMatch(descriptors1, descriptors2, matches, 2);
  //
  // vector<KeyPoint> keypoints1_res, keypoints2_res;
  // surf.downloadKeypoints(keypoints1, keypoints1_res);
  // surf.downloadKeypoints(keypoints2, keypoints2_res);
  //
  // vector<DMatch> goodMatches;
  // vector<Point2f> points1, points2;
  // for (int i = 0; i < min(keypoints1_res.size() - 1, matches.size()); i++) {
  //   if (matches[i][0].distance < 0.6 * matches[i][1].distance &&
  //       ((int) matches[i].size() <= 2 && (int) matches[i].size() > 0)) {
  //       goodMatches.push_back(matches[i][0]);
  //       points1.push_back(keypoints1_res[matches[i][0].queryIdx].pt);
  //       points2.push_back(keypoints2_res[matches[i][0].trainIdx].pt);
  //     }
  // }

  // surf.releaseMemory();
  // matcher.release();
  // img1Gray.release();
  // img2Gray.release();

  // Mat imMatches;
  // drawMatches(im1, keypoints1_res, im2, keypoints2_res, goodMatches, imMatches);
  // Find homography
  // Mat h = findHomography(points2, points1, RANSAC );

  Mat h;
  FileStorage file("/home/donamphuong/ImmersiveTeleoperation/src/stitcher/homography/test.yaml", FileStorage::READ);
  file["homography"] >> h;
  file.release();

  return h;
}

void alignImages(Mat im1, Mat im2) {
  Mat h = homography(im1, im2);
  Mat im2Warped;
  warpPerspective(im2, im2Warped, h, Size(im2.cols * 2, im2.rows));
  Mat aligned(im1.rows, 2 * im1.cols, im1.type());

  // images area in the final stitched image
  Mat imLeftArea = aligned(Rect(0, 0, im1.cols, im1.rows));
  Mat imRightArea = aligned(Rect(im1.cols, 0, im2.cols, im2.rows));

  Mat roiImgLeft = im1(Rect(0, 0, im1.cols, im2.rows));
  Mat roiImgRight = im2Warped(Rect(im1.cols, 0, im2.cols, im2.rows));

  roiImgLeft.copyTo(imLeftArea);
  roiImgRight.copyTo(imRightArea);

  namedWindow("warped", WINDOW_NORMAL);
  resizeWindow("warped", 1024, 600);
  imshow ("warped", aligned);
  waitKey();
}

void initialiseParams(vector<Mat> images, vector<Point> &corners,
                      vector<UMat> &masksWarped, vector<UMat> &imagesWarped,
                      vector<Size> &sizes, vector<UMat> &masks, Ptr<RotationWarper> warper) {
  // the mask of an image is a white rectangle of the same size as the image
  for (int i = 0; i < numImage; i++) {
    masks[i].create(images[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));
  }

  for (int i = 0; i < numImage; i++) {
    Mat K, R;
    calibrations[i].camera_matrix.convertTo(K, CV_32F);
    calibrations[i].rectification.convertTo(R, CV_32F);

    corners[i] = warper->warp(images[i], K, R, INTER_LINEAR, BORDER_REFLECT, imagesWarped[i]);
    sizes[i] = imagesWarped[i].size();
    // imshow("result",imagesWarped[i]);
    // waitKey();
    // save projected mask in masksWarped
    warper->warp(masks[i], K, R, INTER_NEAREST, BORDER_CONSTANT, masksWarped[i]);
  }
}


void warpAndBlend(vector<Mat> images) {
  std::clock_t start;
  double duration;
  start = std::clock();

  //stores top left corners coordinates of each image
  vector<Mat> imagesWarped(numImage);
  vector<UMat> masks(numImage);
  // cv::PlaneWarper warperCreator;
  // Ptr<RotationWarper> warper = warperCreator.create(1.0);

  // initialiseParams(images, corners, masksWarped, imagesWarped, sizes, masks, warper);
  //
  // vector<UMat> imagesWarped_F(numImage);
  // for (int i = 0; i < numImage; i++) {
  //   imagesWarped[i].convertTo(imagesWarped_F[i], CV_32F);
  // }
  //
  // GraphCutSeamFinder seamFinder = GraphCutSeamFinder(GraphCutSeamFinderBase::COST_COLOR_GRAD);
  // // estimating seams
  // seamFinder.find(imagesWarped_F, corners, masksWarped);

  // the mask of an image is a white rectangle of the same size as the image
  for (int i = 0; i < numImage; i++) {
    masks[i].create(images[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));

    if (i != 0) {
      Mat previousImage = imagesWarped[i-1];
      Mat currentImage = images[i];
      Mat h = homography(previousImage, currentImage);
      warpPerspective(currentImage, imagesWarped[i], h, Size(currentImage.cols * 2, currentImage.rows));
    } else {
      imagesWarped[i] = images[i];
    }
  }

  FeatherBlender blender(1.0f);
  blender.prepare(Rect(0, 0, images[0].cols + images[1].cols, images[0].rows + images[0].rows));

  for (int i = 0; i < numImage; i++) {
    Mat imageS;
    imagesWarped[i].convertTo(imageS, CV_16S);

    blender.feed(imageS, masks[i], Point(0, 0));
  }

  Mat result, result_s, result_mask;
  blender.blend(result_s, result_mask);
  result_s.convertTo(result, CV_8U);
  imshow("result",result);
  waitKey();
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"printf: "<< duration <<'\n';

}

void stitch(Mat im1, Mat im2) {
    vector<Mat> images;
    images.push_back(im1);
    images.push_back(im2);
    warpAndBlend(images);
}

void getCalibrationDetails() {
  for (int i = 1; i < 2+1; i++) {
    CalibrationDetails cal;
    std::string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/calibration/camera" + to_string(i) + ".yaml";
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
      std::cout << "Could not open the configuration file" << std::endl;
      exit(-1) ;
    }

    fs["camera_matrix"] >> cal.camera_matrix;
    fs["distortion_coefficients"] >> cal.distortion;
    fs["rectification_matrix"] >> cal.rectification;
    fs["projection_matrix"] >> cal.projection;
    fs.release();

    calibrations.push_back(cal);
  }
}

int main(int argc, char** argv) {
  getCalibrationDetails();
  cv::Mat im1, im2;

  im1 = imread("images/test2.png"/*, IMREAD_GRAYSCALE*/);
  im2 = imread("images/test1.png"/*, IMREAD_GRAYSCALE*/);

  stitch(im1, im2);

  return 0;
}
