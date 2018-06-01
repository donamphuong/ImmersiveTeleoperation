#include "test.h"

// #type def unsigned char pixel

__global__
void add(int n, float *x, float *y) {
  //index of the current thread
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  //stride is the total number of threads in the grid
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < n; i += stride)
    y[i] = x[i] + y[i];
}

__global__
void copy(pixel *src, pixel *dst, pixel *mask, pixel *dst_mask,
          const int src_width,
          const Point mask_tl, const Rect dst_roi,
          const int dst_width, const int mask_width, const int mask_height,
          const int dst_mask_width) {
  //2D index of current thread
  int x_index = blockIdx.x * blockDim.x + threadIdx.x;
  int y_index = blockIdx.y * blockDim.y + threadIdx.y;

  //only valid threads (threads that copy pixel which are inside mask) perform memory I/O
  //location of top left corner of image in the canvas
  int dx = mask_tl.x - dst_roi.x;
  int dy = mask_tl.y - dst_roi.y;

  if (x_index < dx + mask_width && y_index > dy - mask_height) {
    // for (int y = 0; y < src_height; ++y) {
      //pixel location = row * row_size + col
      int src_pixel = y_index * src_width + x_index;
      int dst_pixel = (dy + y_index) * dst_width + (dx + x_index);
      int mask_pixel = y_index * mask_width + x_index;
      int dst_mask_pixel = (dy + y_index) * dst_mask_width + (dx + x_index);
  
      // for (int x = 0; x < src_width; ++x)
      // {
          if (mask[mask_pixel])
              dst[dst_pixel] = src[src_pixel];
          dst_mask[dst_mask_pixel] |= mask[mask_pixel];
      // }
    // }  
  }
}

void testCopy(const Mat &src, const Mat &dst, const Mat &mask, const Mat &dst_mask, const Point &tl, const Rect &dst_roi) {
  const int srcBytes = src.step * src.rows;
  const int dstBytes = dst.step * dst.rows;
  const int maskBytes = mask.step * mask.rows;
  const int dstMaskBytes = dst_mask.step * dst_mask.rows;

  pixel *d_src, *d_dst, *d_mask, *d_dst_mask;

  //Allocate device memory
  cudaMalloc<pixel>(&d_src, srcBytes);
  cudaMalloc<pixel>(&d_dst, dstBytes);
  cudaMalloc<pixel>(&d_mask, maskBytes);
  cudaMalloc<pixel>(&d_dst_mask, dstMaskBytes);

  //Copy data from OpenCV input image to device memory
  cudaMemcpy(d_src, src.ptr(), srcBytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_mask, mask.ptr(), maskBytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_dst_mask, dst_mask.ptr(), dstMaskBytes, cudaMemcpyHostToDevice);

  //Sprcify a reasonable block size
  const dim3 block(16, 16);

  //Calculate frid size to cover the whole iamge
  const dim3 grid((dst.cols + block.x - 1)/block.x, (dst.rows + block.y - 1)/block.y);

  copy<<<grid, block>>>(d_src, d_dst, d_mask, d_dst_mask, src.cols, tl, dst_roi, dst.cols, mask.cols, mask.rows, dst_mask.cols);
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

__global__ 
void resizeCudaKernel( unsigned char* input,
  unsigned char* output,
  const int outputWidth,
  const int outputHeight,
  const int inputWidthStep,
  const int outputWidthStep,
  const float pixelGroupSizeX,
  const float pixelGroupSizeY,
  const int inputChannels)
{
//2D Index of current thread
const int outputXIndex = blockIdx.x * blockDim.x + threadIdx.x;
const int outputYIndex = blockIdx.y * blockDim.y + threadIdx.y;

//Only valid threads perform memory I/O
if((outputXIndex<outputWidth) && (outputYIndex<outputHeight))
{
  // Starting location of current pixel in output
  int output_tid  = outputYIndex * outputWidthStep + (outputXIndex * inputChannels);

  // Compute the size of the area of pixels to be resized to a single pixel
  const float pixelGroupArea = pixelGroupSizeX * pixelGroupSizeY;

  // Compute the pixel group area in the input image
  const int intputXIndexStart = int(outputXIndex * pixelGroupSizeX);
  const int intputXIndexEnd = int(intputXIndexStart + pixelGroupSizeX);
  const float intputYIndexStart = int(outputYIndex * pixelGroupSizeY);
  const float intputYIndexEnd = int(intputYIndexStart + pixelGroupSizeY);

  if(inputChannels==1) { // grayscale image
    float channelSum = 0;
    for(int intputYIndex=intputYIndexStart; intputYIndex<intputYIndexEnd; ++intputYIndex) {
      for(int intputXIndex=intputXIndexStart; intputXIndex<intputXIndexEnd; ++intputXIndex) {
        int input_tid = intputYIndex * inputWidthStep + intputXIndex;
        channelSum += input[input_tid];
      }
    }
    output[output_tid] = static_cast<unsigned char>(channelSum / pixelGroupArea);
  } else if(inputChannels==3) { // RGB image
    float channel1stSum = 0;
    float channel2stSum = 0;
    float channel3stSum = 0;
    for(int intputYIndex=intputYIndexStart; intputYIndex<intputYIndexEnd; ++intputYIndex) {
      for(int intputXIndex=intputXIndexStart; intputXIndex<intputXIndexEnd; ++intputXIndex) {
        // Starting location of current pixel in input
        int input_tid = intputYIndex * inputWidthStep + intputXIndex * inputChannels;
        channel1stSum += input[input_tid];
        channel2stSum += input[input_tid+1];
        channel3stSum += input[input_tid+2];
      }
    }
    output[output_tid] = static_cast<unsigned char>(channel1stSum / pixelGroupArea);
    output[output_tid+1] = static_cast<unsigned char>(channel2stSum / pixelGroupArea);
    output[output_tid+2] = static_cast<unsigned char>(channel3stSum / pixelGroupArea);
  } else { 
  }
}
}

void downscaleCuda(const cv::Mat& input, cv::Mat& output)
{
	//Calculate total number of bytes of input and output image
	const int inputBytes = input.step * input.rows;
	const int outputBytes = output.step * output.rows;
 
	unsigned char *d_input, *d_output;
 
	//Allocate device memory
	cudaMalloc<unsigned char>(&d_input,inputBytes);
	cudaMalloc<unsigned char>(&d_output,outputBytes);
 
  clock_t start = clock();
	//Copy data from OpenCV input image to device memory
	cudaMemcpy(d_input,input.ptr(),inputBytes,cudaMemcpyHostToDevice);
  double duration = (clock() - start) / (double) CLOCKS_PER_SEC;
	cout << "Copying memory takes " << duration << " secs" << endl;

  start = clock();

	//Specify a reasonable block size
	const dim3 block(16,16);
 
	//Calculate grid size to cover the whole image
	const dim3 grid((output.cols + block.x - 1)/block.x, (output.rows + block.y - 1)/block.y);
 
	// Calculate how many pixels in the input image will be merged into one pixel in the output image
	const float pixelGroupSizeY = float(input.rows) / float(output.rows);
	const float pixelGroupSizeX = float(input.cols) / float(output.cols);
 
	//Launch the size conversion kernel
	resizeCudaKernel<<<grid,block>>>(d_input,d_output,output.cols,output.rows,input.step,output.step, pixelGroupSizeX, pixelGroupSizeY, input.channels());
 
	duration = (clock() - start) / (double) CLOCKS_PER_SEC;
	cout << "OpenCv Gpu code ran in:" << duration << " secs" << endl;
 
	//Synchronize to check for any kernel launch errors
	cudaDeviceSynchronize();
 
	//Copy back data from destination device meory to OpenCV output image
	cudaMemcpy(output.ptr(),d_output,outputBytes,cudaMemcpyDeviceToHost);
 
	//Free the device memory
	cudaFree(d_input);
	cudaFree(d_output);
	//cudaDeviceReset(),"CUDA Device Reset Failed");
}
