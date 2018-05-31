#include "test.h"

__global__
void add(int n, float *x, float *y) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < n; i += stride)
    y[i] = x[i] + y[i];
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
  int blockSize = 256;
  int numBlocks = (N + blockSize - 1) / blockSize;
  clock_t start = clock();
  add<<<numBlocks, blockSize>>>(N, x, y);
  cout << "time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;


  cudaDeviceSynchronize();

  // Free memory
  cudaFree(x);
  cudaFree(y);
}
