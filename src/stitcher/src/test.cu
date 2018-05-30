#include "test.h"

__global__
void add(int n, float *x, float *y) {
  for (int i = 0; i < n; i++) {
    y[i] = x[i] + y[i];
  }
}

void test() {
  int N = 1 << 20;
  float *x, *y;

  //Allocate Unified memory
  cudaMallocManaged(&x, N*sizeof(float));
  cudaMallocManaged(&y, N*sizeof(float));

  //Initialise x and y arrays
  for (int i = 0; i < N; i++) {
    x[i] = 1.0f;
    y[i] = 2.0f;
  }

  //Run kernel on 1M elements on the GPU
  add<<<1, 1>>>(N, x, y);

  cudaDeviceSynchronize();

  // Free memory
  cudaFree(x);
  cudaFree(y);
}
