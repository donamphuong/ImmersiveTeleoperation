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

  vector<Mat> images_warped_s(numImage);

  #ifdef DEBUG
    start = clock();
  #endif

  //build another canvas when the first image in the panorama is inputted
  blender->prepare(composedCorners, updatedSizes);

  for (int img_idx = 0; img_idx < numImage; img_idx++) {
    Mat img, img_warped;
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

    img_warped.convertTo(images_warped_s[img_idx], CV_16S);

    // Compensate exposure
    // compensator->apply(img_idx, composedCorners[img_idx], img_warped, composed_warped_masks[img_idx]);

    dilate(composed_warped_masks[img_idx], dilated_mask, Mat());
    resize(dilated_mask, seam_mask, composed_warped_masks[img_idx].size(), 0, 0, INTER_LINEAR_EXACT);

    composed_warped_masks[img_idx] = seam_mask & composed_warped_masks[img_idx];

    // Blend the current image
    #ifdef DEBUG
      startWarp = clock();
    #endif
    blender->feed(images_warped_s[img_idx], composed_warped_masks[img_idx], composedCorners[img_idx]);
    #ifdef DEBUG
      warpTime += (clock() - startWarp) / (double) CLOCKS_PER_SEC;
    #endif
  }

  images_warped_s.clear();

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Loop time: " << duration << "\n";
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  Mat result_s, result_mask;
  blender->blend(result_s, result_mask);

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

  vector<UMat> images_warped(numImage);
  vector<Size> sizes(numImage);
  vector<Point> corners(numImage);

  #ifdef DEBUG
    start = clock();
  #endif

  // Warping image based on precomputed spherical map
  for (int i = 0; i < numImage; ++i) {
    Rect image_roi = sphericalImageROI[i];
    images_warped[i].create(image_roi.height + 1, image_roi.width + 1, images[i].type());
    remap(images[i], images_warped[i], sphericalImageUXMap[i], sphericalImageUYMap[i], INTER_LINEAR, BORDER_REFLECT);

    corners[i] = image_roi.tl();
    sizes[i] = images_warped[i].size();
  }

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Warping time: " << duration << "\n";
  #endif

  vector<UMat> images_warped_f(numImage);
  for (int i = 0; i < numImage; ++i) {
    images_warped[i].convertTo(images_warped_f[i], CV_32F);
  }

  #ifdef DEBUG
    start = clock();
  #endif

    compensator->feed(corners, images_warped, masks_warped);

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Exposure Compensating Time: " << duration << endl;
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

    seam_finder->find(images_warped_f, corners, masks_warped);

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Finding seam time: " << duration << "\n";
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Conversion time: " << duration << endl;
  #endif

  // Release unused memory
  images.clear();
  images_warped.clear();
  images_warped_f.clear();
  corners.clear();
  sizes.clear();
  masks_warped.clear();

  compose(full_images, result);
}
