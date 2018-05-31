#include "precomp.cpp"
#include <tbb/tbb.h>
#include <tbb/task_group.h>
#include <tbb/parallel_invoke.h>

#define DEBUG

void compose(vector<Mat> full_images, Mat &result) {
  #ifdef DEBUG
    clock_t start, startWarp;
    double duration, warpTime;
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  // //build another canvas when the first image in the panorama is inputted
  // blender->prepare(composedCorners, updatedSizes);

  //Reset the visibility of the canvas
  dst.setTo(Scalar::all(0));
  dst_mask.setTo(Scalar::all(0));

  for (int img_idx = 0; img_idx < numImage; img_idx++) {
    Mat img, img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask;
    // Read image and resize it if necessary
    img = full_images[img_idx];

    Mat K, R;
    calibrations[img_idx].camera_matrix.convertTo(K, CV_32F);
    calibrations[img_idx].rectification.convertTo(R, CV_32F);

    // Warping image based on precomputed spherical map
    Rect image_roi = composedImageROI[img_idx];
    img_warped.create(image_roi.height + 1, image_roi.width + 1, img.type());
    remap(img, img_warped, composedImageUXMap[img_idx], composedImageUYMap[img_idx], INTER_LINEAR, BORDER_REFLECT);

    img_warped.convertTo(img_warped_s, CV_16S);
    //
    // // Compensate exposure
    // // compensator->apply(img_idx, composedCorners[img_idx], img_warped, composed_warped_masks[img_idx]);
    //
    // dilate(composed_warped_masks[img_idx], dilated_mask, Mat());
    // resize(dilated_mask, seam_mask, composed_warped_masks[img_idx].size(), 0, 0, INTER_LINEAR_EXACT);
    //
    // composed_warped_masks[img_idx] = seam_mask & composed_warped_masks[img_idx];

    // Blend the current image
    #ifdef DEBUG
      startWarp = clock();
    #endif
    // blender->feed(img_warped_s, composed_warped_masks[img_idx], composedCorners[img_idx]);
    int dx = composedCorners[img_idx].x - dst_roi.x;
    int dy = composedCorners[img_idx].y - dst_roi.y;

    for (int y = 0; y < img_warped_s.rows; ++y) {
      const Point3_<short> *src_row = img_warped_s.ptr<Point3_<short> >(y);
      Point3_<short> *dst_row = dst.ptr<Point3_<short> >(dy + y);
      const uchar *mask_row = composed_warped_masks[img_idx].ptr<uchar>(y);
      uchar *dst_mask_row = dst_mask.ptr<uchar>(dy + y);

      for (int x = 0; x < img_warped_s.cols; ++x)
      {
          if (mask_row[x])
              dst_row[dx + x] = src_row[x];
          dst_mask_row[dx + x] |= mask_row[x];
      }
    }

    #ifdef DEBUG
      warpTime += (clock() - startWarp) / (double) CLOCKS_PER_SEC;
    #endif
  }
  cout << "hllo" << endl;

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Loop time: " << duration << "\n";
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  Mat result_s, result_mask;
  // blender->blend(result_s, result_mask);
  UMat mask;
  compare(result_mask, 0, mask, CMP_EQ);
  result_s.setTo(Scalar::all(0), mask);
  result_s = (dst);
  result_mask = (dst_mask);
  dst.release();
  dst_mask.release();

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Blending time: " << duration << "\n";
    cout << "Warping 2nd time: " << warpTime << "\n";
  #endif

  result_s.convertTo(result, CV_8U);
}

Mat stitch(const vector<Mat> &full_images, Mat &result) {
  #ifdef DEBUG
    cout << endl << "START" << endl;
    clock_t start, startWarp;
    double duration, warpTime;
  #endif

  Mat full_img, img;
  vector<Mat> images(numImage);

  #ifdef DEBUG
    start = clock();
  #endif


  for (int i = 0; i < numImage; ++i) {
    full_img = full_images[i];

    resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);

    images[i] = img.clone();
  }

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Reading images " << duration << endl;
  #endif

  full_img.release();
  img.release();

  // Release unused memory
  images.clear();

  compose(full_images, result);
}
