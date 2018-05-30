#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "../details.cpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

double work_scale = 0.5;
double seam_scale = 1;
double compose_scale = 1;
double compose_work_aspect = 1;
double seam_work_aspect = 1;
float warped_image_scale;
Size image_scale_size = Size(422, 237);
Size image_size = Size(1920, 1080);

vector<UMat> sphericalImageUXMap(numImage);
vector<UMat> sphericalImageUYMap(numImage);
vector<Rect> sphericalImageROI(numImage);

vector<UMat> composedImageUXMap(numImage);
vector<UMat> composedImageUYMap(numImage);
vector<Rect> composedImageROI(numImage);
vector<UMat> composedMaskUXMap(numImage);
vector<UMat> composedMaskUYMap(numImage);
vector<Rect> composedMaskROI(numImage);

vector<Point> composedCorners(numImage);
vector<Size> updatedSizes(numImage);
vector<UMat> masks_warped(numImage);

Ptr<WarperCreator> warper_creator;
Ptr<RotationWarper> warper;
Ptr<ExposureCompensator> compensator;
Ptr<SeamFinder> seam_finder;
Ptr<Blender> blender;

void initBlender();
void buildComposedMaps();
void buildSphericalMaps();
void precomp();
