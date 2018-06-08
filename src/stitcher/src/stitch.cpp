#include "headers/stitch.hpp"

void place_images(int img_idx, Mat &img_warped_s) {
  int dx = composedCorners[img_idx].x - dst_roi.x;
  int dy = composedCorners[img_idx].y - dst_roi.y;

  for (int y = 0; y < img_warped_s.rows; ++y) {
    const Point3_<short> *src_row = img_warped_s.ptr<Point3_<short> >(y);
    Point3_<short> *dst_row = dst.ptr<Point3_<short> >(dy + y);
    const uchar *mask_row = composed_warped_masks[img_idx].ptr<uchar>(y);
    uchar *dst_mask_row = dst_mask.ptr<uchar>(dy + y);

    for (int x = 0; x < img_warped_s.cols; ++x) {
        if (mask_row[x])
            dst_row[dx + x] = src_row[x];
        dst_mask_row[dx + x] |= mask_row[x];
    }
  }
}

void feather_blend(int img_idx, Mat &img_warped_s, Mat &weight_map) {
  int dx = composedCorners[img_idx].x - dst_roi.x;
  int dy = composedCorners[img_idx].y - dst_roi.y;

  for (int y = 0; y < img_warped_s.rows; ++y) {
    const Point3_<short>* src_row = img_warped_s.ptr<Point3_<short> >(y);
    Point3_<short>* dst_row = dst.ptr<Point3_<short> >(dy + y);
    const float* weight_row = weight_map.ptr<float>(y);
    float* dst_weight_row = dst_weight_map.ptr<float>(dy + y);

    for (int x = 0; x < img_warped_s.cols; ++x) {
        dst_row[dx + x].x += static_cast<short>(src_row[x].x * weight_row[x]);
        dst_row[dx + x].y += static_cast<short>(src_row[x].y * weight_row[x]);
        dst_row[dx + x].z += static_cast<short>(src_row[x].z * weight_row[x]);
        dst_weight_row[dx + x] += weight_row[x];
    }
  }
}

void normalize_blended_image() {
  for (int y = 0; y < dst.rows; ++y) {
    Point3_<short> *row = dst.ptr<Point3_<short> >(y);
    const float *weight_row = dst_weight_map.ptr<float>(y);

    for (int x = 0; x < dst.cols; ++x) {
      row[x].x = static_cast<short>(row[x].x / (weight_row[x] + WEIGHT_EPS));
      row[x].y = static_cast<short>(row[x].y / (weight_row[x] + WEIGHT_EPS));
      row[x].z = static_cast<short>(row[x].z / (weight_row[x] + WEIGHT_EPS));
    }
  }
}

void stitch(vector<Mat> &full_images, Mat &result) {
  #ifdef DEBUG
    cout << endl << "START" << endl;
    clock_t start, startWarp;
    double duration, warpTime;
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  //Reset the visibility of the canvas and the weight map to be zero
  //a blank canvas should be initialised when images need to be stitched
  dst.setTo(Scalar::all(0));
  dst_mask.setTo(Scalar::all(0));
  dst_weight_map.setTo(0);

  vector<Mat> maps(numImage);
  for (int i = 0; i < numImage; i++) {
    maps[i] = weight_maps[i].getMat(ACCESS_READ);
  }

  Mat img, img_warped, img_warped_s;
  for (int img_idx = 0; img_idx < numImage; img_idx++) {
    // Read image and resize it if necessary
    img = full_images[img_idx];

    // Warping image based on precomputed spherical map
    Rect image_roi = composedImageROI[img_idx];
    img_warped.create(image_roi.height + 1, image_roi.width + 1, img.type());
    remap(img, img_warped, composedImageUXMap[img_idx], composedImageUYMap[img_idx], INTER_LINEAR, BORDER_REFLECT);

    img_warped.convertTo(img_warped_s, CV_16S);

    // Blend the current image
    #ifdef DEBUG
      startWarp = clock();
    #endif

    float sharpness = 0.02f;
    feather_blend(img_idx, img_warped_s, maps[img_idx]);
    // place_images(img_idx, img_warped_s);

    #ifdef DEBUG
      warpTime += (clock() - startWarp) / (double) CLOCKS_PER_SEC;
    #endif
    img_warped.convertTo(img_warped_s, CV_16S);

    // place_image(img_idx, img_warped_s);
    feather_blend(img_idx, img_warped_s, maps);
  }

  img.release();
  img_warped.release();
  img_warped_s.release();

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Loop time: " << duration << "\n";
    cout << "Input images time: " << warpTime << endl;
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  normalize_blended_image();
  compare(dst_weight_map, WEIGHT_EPS, dst_mask, CMP_GT);

  UMat mask;
  compare(dst_mask, 0, mask, CMP_EQ);
  dst.setTo(Scalar::all(0), mask);
  mask.release();

  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Blending time: " << duration << "\n";
  #endif

  dst.convertTo(result, CV_8U);
}
