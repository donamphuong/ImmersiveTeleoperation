#include "test.h"

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


void remapCuda(pixel *src, pixel *dst, UMat UXMap, UMat UYMat) {

}