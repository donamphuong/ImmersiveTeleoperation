#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <math.h>
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
#include "details.cpp"


#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::detail;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;

Mat homography(Mat im1, Mat im2) {
  int minHessain = 400;
  Mat gray1, gray2;
  cvtColor(im1, gray1, COLOR_BGR2GRAY);
  cvtColor(im2, gray2, COLOR_BGR2GRAY);

  // Copy image into GPU memory
  cuda::GpuMat img1Gray(gray1);
  cuda::GpuMat img2Gray(gray2);

  // Variables to store keypoints and descriptors
  cuda::GpuMat keypoints1, keypoints2;
  cuda::GpuMat descriptors1, descriptors2;

  // Detect ORB features and compute descriptors.
  cuda::SURF_CUDA surf(minHessain);
  surf(img1Gray, cuda::GpuMat(), keypoints1, descriptors1);
  surf(img2Gray, cuda::GpuMat(), keypoints2, descriptors2);

  // Match features.
  vector<vector<DMatch> > matches;
  Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher();
  matcher->knnMatch(descriptors1, descriptors2, matches, 2);

  vector<KeyPoint> keypoints1_res, keypoints2_res;
  surf.downloadKeypoints(keypoints1, keypoints1_res);
  surf.downloadKeypoints(keypoints2, keypoints2_res);

  vector<DMatch> goodMatches;
  vector<Point2f> points1, points2;
  for (int i = 0; i < min(keypoints1_res.size() - 1, matches.size()); i++) {
    if (matches[i][0].distance < 0.6 * matches[i][1].distance &&
        ((int) matches[i].size() <= 2 && (int) matches[i].size() > 0)) {
        goodMatches.push_back(matches[i][0]);
        points1.push_back(keypoints1_res[matches[i][0].queryIdx].pt);
        points2.push_back(keypoints2_res[matches[i][0].trainIdx].pt);
      }
  }

  surf.releaseMemory();
  matcher.release();
  img1Gray.release();
  img2Gray.release();

  // Mat imMatches;
  // drawMatches(im1, keypoints1_res, im2, keypoints2_res, goodMatches, imMatches);
  // imshow("matches", imMatches);
  // waitKey();
  // Find homography
  Mat h = findHomography(points2, points1, RANSAC );
  return h;
}

Mat affineTransform(Mat im1, Mat im2) {
  int minHessain = 400;
  Mat gray1, gray2;
  cvtColor(im1, gray1, COLOR_BGR2GRAY);
  cvtColor(im2, gray2, COLOR_BGR2GRAY);

  // Copy image into GPU memory
  cuda::GpuMat img1Gray(gray1);
  cuda::GpuMat img2Gray(gray2);

  // Variables to store keypoints and descriptors
  cuda::GpuMat keypoints1, keypoints2;
  cuda::GpuMat descriptors1, descriptors2;

  // Detect ORB features and compute descriptors.
  cuda::SURF_CUDA surf(minHessain);
  surf(img1Gray, cuda::GpuMat(), keypoints1, descriptors1);
  surf(img2Gray, cuda::GpuMat(), keypoints2, descriptors2);

  // Match features.
  vector<vector<DMatch> > matches;
  Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher();
  matcher->knnMatch(descriptors1, descriptors2, matches, 2);

  vector<KeyPoint> keypoints1_res, keypoints2_res;
  surf.downloadKeypoints(keypoints1, keypoints1_res);
  surf.downloadKeypoints(keypoints2, keypoints2_res);

  vector<DMatch> goodMatches;
  vector<Point2f> points1, points2;
  for (int i = 0; i < min(keypoints1_res.size() - 1, matches.size()); i++) {
    if (matches[i][0].distance < 0.45 * matches[i][1].distance &&
        ((int) matches[i].size() <= 2 && (int) matches[i].size() > 0)) {
        goodMatches.push_back(matches[i][0]);
        points1.push_back(keypoints1_res[matches[i][0].queryIdx].pt);
        points2.push_back(keypoints2_res[matches[i][0].trainIdx].pt);
      }
  }

  surf.releaseMemory();
  matcher.release();
  img1Gray.release();
  img2Gray.release();

  // Mat imMatches;
  // drawMatches(im1, keypoints1_res, im2, keypoints2_res, goodMatches, imMatches);
  // imshow("matches", imMatches);
  // waitKey();
  // Find homography

  return estimateRigidTransform(points1, points2, true);
}


void alignImages(Mat im1, Mat im2) {
  std::clock_t start;

  double duration;

  start = std::clock();

  // Mat h = homography(im1, im2);
  Mat h = affineTransform(im1, im2);
  cout << h << endl;
  // h = h.inv();
  Mat im2Warped;
  // warpPerspective(im2, im2Warped, h, Size(im2.cols * 3, im2.rows));
  warpAffine(im2, im2Warped, h, Size(im2.cols * 3, im2.rows));
  Mat aligned(im1.rows, 2 * im1.cols, im1.type());

  // images area in the final stitched image
  Mat imLeftArea = aligned(Rect(0, 0, im1.cols, im1.rows));
  Mat imRightArea = aligned(Rect(im1.cols, 0, im2.cols, im2.rows));

  Mat roiImgLeft = im1(Rect(0, 0, im1.cols, im2.rows));
  Mat roiImgRight = im2Warped(Rect(im1.cols, 0, im2.cols, im2.rows));

  roiImgLeft.copyTo(imLeftArea);
  roiImgRight.copyTo(imRightArea);
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

  std::cout<<"printf: "<< duration <<'\n';

  namedWindow("warped", WINDOW_NORMAL);
  resizeWindow("warped", 1024, 600);
  imshow ("warped", roiImgRight);
  waitKey();
}

int main(int argc, char** argv) {
  cv::Mat im1, im2, im1Warped, im2Warped;
  homography();
  getCalibrationDetails(numImage);
  im1 = imread("test32.png");
  im2 = imread("test23.png");
  Ptr<WarperCreator> warperCreator = makePtr<cv::CylindricalWarper>();
  Ptr<RotationWarper> warper = warperCreator->create(static_cast<float>(1000));

    // Mat K, R, pose = Mat::eye(3, 3, CV_32F);
    // vector<Mat> rotations, translations, normals;
    // decomposeHomographyMat(homographies[0], calibrations[1].camera_matrix, rotations, translations, normals);
    // for (int i = 0; i < rotations.size(); i++) {
    //   cout << "Rotation " << i << endl << rotations[i] << endl;
    // }
    //
    // decomposeHomographyMat(homographies[0], calibrations[0].camera_matrix, rotations, translations, normals);
    // for (int i = 0; i < rotations.size(); i++) {
    //   cout << "Rotation " << i << endl << rotations[i] << endl;
    // }


    // pose.colRange(0, 3).rowRange(0, 3).convertTo(R, CV_32F);
    // calibrations[1].camera_matrix.convertTo(K, CV_32F);
    //
    // warper->warp(im2, K, R, INTER_LINEAR, BORDER_REFLECT, im2Warped);

  // Mat h = affineTransform(im1Warped, im2Warped);
  // std::string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/affine/test.yaml";
  // FileStorage file(filename, FileStorage::WRITE);
  //
  // file << "affine" << h;
  // file.release();
  // cout << "HELLO" << endl;

  // namedWindow("warped", WINDOW_NORMAL);
  // resizeWindow("warped", 1024, 600);
  // imshow ("warped", im2Warped);
  // waitKey();

  return 0;
}
