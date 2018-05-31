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

using namespace std;
// using namespace cv::cuda;

__global__
void add(int n, float *x, float *y);
void test();
