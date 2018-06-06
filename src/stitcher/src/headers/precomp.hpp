#ifndef PRECOMP_HPP_
#define PRECOMP_HPP_

#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include "details.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

const Size image_size = Size(1920, 1080);

vector<UMat> composedImageUXMap(numImage);
vector<UMat> composedImageUYMap(numImage);
vector<Rect> composedImageROI(numImage);

vector<Point> composedCorners(numImage);
vector<Size> updatedSizes(numImage);
vector<Mat> composed_warped_masks(numImage);

Ptr<WarperCreator> warper_creator;
Ptr<RotationWarper> warper;
Mat dst;
Mat dst_mask;
Rect dst_roi;
Mat dst_weight_map;
vector<UMat> weight_maps(numImage);

void buildComposedMaps();
void precomp();

#endif