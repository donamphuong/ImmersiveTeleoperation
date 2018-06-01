#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <tbb/tbb.h>
#include <opencv2/opencv.hpp>
#include <cuda.h>
#include <cuda_runtime.h>

typedef unsigned char pixel;

using namespace std;
using namespace cv;
// using namespace cv::cuda;

__global__
void add(int n, float *x, float *y);
__global__
void copy(pixel *src, pixel *dst);
__global__
void resizeCudaKernel( unsigned char* input,
  unsigned char* output,
  const int outputWidth,
  const int outputHeight,
  const int inputWidthStep,
  const int outputWidthStep,
  const float pixelGroupSizeX,
  const float pixelGroupSizeY,
  const int inputChannels);
void test();
void downscaleCuda(const cv::Mat& input, cv::Mat& output);