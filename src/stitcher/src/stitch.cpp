#include "precomp.cpp"
#include <tbb/tbb.h>
#include <tbb/task_group.h>
#include <tbb/parallel_invoke.h>

#define DEBUG

Mat stitch(const vector<Mat> &full_images, Mat &result) {
  #ifdef DEBUG
    cout << endl << "START" << endl;
    clock_t start, startWarp;
    double duration, warpTime;
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  vector<Mat> maps(numImage);
  for (int i = 0; i < numImage; i++) {
    maps[i] = weight_maps[i].getMat(ACCESS_READ);
  }
  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Conversion time: " << duration << "\n";
  #endif

  //Reset the visibility of the canvas
  dst.setTo(Scalar::all(0));
  dst_mask.setTo(Scalar::all(0));

  #ifdef DEBUG
    start = clock();
  #endif
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

    // Blend the current image

    int dx = composedCorners[img_idx].x - dst_roi.x;
    int dy = composedCorners[img_idx].y - dst_roi.y;

    for (int y = 0; y < img_warped_s.rows; ++y)
    {
        const Point3_<short>* src_row = img_warped_s.ptr<Point3_<short> >(y);
        Point3_<short>* dst_row = dst.ptr<Point3_<short> >(dy + y);
        const float* weight_row = maps[img_idx].ptr<float>(y);
        float* dst_weight_row = dst_weight_map.ptr<float>(dy + y);

        for (int x = 0; x < img_warped_s.cols; ++x)
        {
            dst_row[dx + x].x += static_cast<short>(src_row[x].x * weight_row[x]);
            dst_row[dx + x].y += static_cast<short>(src_row[x].y * weight_row[x]);
            dst_row[dx + x].z += static_cast<short>(src_row[x].z * weight_row[x]);
            dst_weight_row[dx + x] += weight_row[x];
        }
    }
  }
  #ifdef DEBUG
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Loop time: " << duration << "\n";
  #endif

  #ifdef DEBUG
    start = clock();
  #endif

  float WEIGHT_EPS = 1e-5f;
  normalizeUsingWeightMap(dst_weight_map, dst);
  #ifdef DEBUG
  duration = (clock() - start) / (double) CLOCKS_PER_SEC;
  cout << "Blending time: " << duration << "\n";
  #endif
  compare(dst_weight_map, WEIGHT_EPS, dst_mask, CMP_GT);
  UMat mask;
  compare(dst_mask, 0, mask, CMP_EQ);
  dst.setTo(Scalar::all(0), mask);



  dst.convertTo(result, CV_8U);
}
