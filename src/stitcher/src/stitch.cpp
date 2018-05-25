#include "headers/stitch.hpp"

void initBlender() {
  // Update corners and sizes
  for (int i = 0; i < numImage; ++i) {
      // Update corner and size
      Mat K, R;
      calibrations[i].camera_matrix.convertTo(K, CV_32F);
      calibrations[i].rectification.convertTo(R, CV_32F);
      Rect roi = warper->warpRoi(image_size, K, R);
      composedCorners[i] = roi.tl();
      updatedSizes[i] = roi.size();
  }

  float blend_strength = 5;
  blender = Blender::createDefault(Blender::MULTI_BAND, false);
  Size dst_sz = resultRoi(composedCorners, updatedSizes).size();
  float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;

  MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
  mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
}

// void buildComposedMaps() {
//   // Compute relative scales
//   compose_work_aspect = compose_scale / work_scale;
//
//   // Update warped image scale
//   warped_image_scale *= static_cast<float>(compose_work_aspect);
//   warper = warper_creator->create(warped_image_scale);
//
//   for (int i = 0; i < numImage; ++i) {
//       Mat_<float> K, R;
//       calibrations[i].camera_matrix.convertTo(K, CV_32F);
//       calibrations[i].rectification.convertTo(R, CV_32F);
//
//       composedImageROI[i] = warper->buildMaps(image_size, K, R, composedImageUXMap[i], composedImageUYMap[i]);
//       composedMaskROI[i] = warper->buildMaps(image_size, K, R, composedMaskeUXMap[i], composedMaskUYMap[i]);
//   }
// }

void buildSphericalMaps() {
  //Initialising image mask
  vector<UMat> masks(numImage);
  // Preapre images masks
  for (int i = 0; i < numImage; ++i) {
    masks[i].create(image_size, CV_8U);
    masks[i].setTo(Scalar::all(255));
  }

  for (int i = 0; i < numImage; i++) {
    Mat_<float> K, R;
    calibrations[i].camera_matrix.convertTo(K, CV_32F);
    calibrations[i].rectification.convertTo(R, CV_32F);
    float swa = (float)seam_work_aspect;
    K(0,0) *= swa; K(0,2) *= swa;
    K(1,1) *= swa; K(1,2) *= swa;

    sphericalImageROI[i] = warper->buildMaps(image_scale_size, K, R, sphericalImageUXMap[i], sphericalImageUYMap[i]);

    //Warping mask based on computed spherical map
    UMat maskUXMap, maskUYMap;
    Rect mask_roi = warper->buildMaps(image_scale_size, K, R, maskUXMap, maskUYMap);
    masks_warped[i].create(mask_roi.height + 1, mask_roi.width + 1, masks[i].type());
    remap(masks[i], masks_warped[i], maskUXMap, maskUYMap, INTER_NEAREST, BORDER_CONSTANT);
  }

}

void precomp() {
  if (calibrations.empty()) {
    cout << "Calibration details have not been loaded yet" << endl;
    exit(ERROR);
  }

  double work_megapix = 0.6;
  double seam_megapix = 0.1;

  work_scale = min(1.0, sqrt(work_megapix * 1e6 / image_size.area()));
  seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / image_size.area()));
  seam_work_aspect = seam_scale / work_scale;

  //TODO: Using constant K
  warped_image_scale = calibrations[0].camera_matrix.at<double>(0, 0);

  // Warp images and their masks
  warper_creator = makePtr<cv::SphericalWarperGpu>();
  if (!warper_creator) {
      cout << "Can't create cylindrical warper" << endl;
      exit(ERROR);
  }
  warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

  //Initialise exposure compensator
  compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN);

  //Initialise seam finder
  seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
  if (!seam_finder) {
      cout << "Can't create graph cut seam finder" << endl;
      exit(ERROR);
  }

  //Initialise image blender
  initBlender();

  buildSphericalMaps();
  // buildComposedMaps();
}

Mat stitch(const vector<Mat> full_images, Mat &result) {
  #ifdef DEBUG
    cout << endl << "START" << endl;
    int64 start, startWarp;
    double duration, warpTime;
  #endif

  Mat full_img, img;
  vector<Mat> images(numImage);
  vector<Size> full_img_sizes(numImage);

  #ifdef DEBUG
    start = getTickCount();
  #endif

  for (int i = 0; i < numImage; ++i) {
      full_img = full_images[i];
      full_img_sizes[i] = full_img.size();

      resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
      resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);

      images[i] = img.clone();
  }

  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Reading images " << duration << endl;
  #endif

  full_img.release();
  img.release();

  vector<UMat> images_warped(numImage);
  vector<Size> sizes(numImage);
  vector<Point> corners(numImage);

  #ifdef DEBUG
    start = getTickCount();
  #endif
  //Warping image based on precomputed spherical map
  for (int i = 0; i < numImage; ++i) {
    Rect image_roi = sphericalImageROI[i];
    images_warped[i].create(image_roi.height + 1, image_roi.width + 1, images[i].type());
    remap(images[i], images_warped[i], sphericalImageUXMap[i], sphericalImageUYMap[i], INTER_LINEAR, BORDER_REFLECT);

    corners[i] = image_roi.tl();
    sizes[i] = images_warped[i].size();
  }
  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Warping time: " << duration << "\n";
  #endif

  vector<UMat> images_warped_f(numImage);
  for (int i = 0; i < numImage; ++i) {
    images_warped[i].convertTo(images_warped_f[i], CV_32F);
  }

  #ifdef DEBUG
    start = getTickCount();
  #endif

    compensator->feed(corners, images_warped, masks_warped);

  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Exposure Compensating Time: " << duration << endl;
  #endif

  #ifdef DEBUG
    start = getTickCount();
  #endif

    seam_finder->find(images_warped_f, corners, masks_warped);

  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Finding seam time: " << duration << "\n";
  #endif

  #ifdef DEBUG
    start = getTickCount();
  #endif

  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Conversion time: " << duration << endl;
  #endif

  // Release unused memory
  images.clear();
  images_warped.clear();
  images_warped_f.clear();
  corners.clear();
  sizes.clear();

  Mat img_warped, img_warped_s;
  Mat dilated_mask, seam_mask, mask, mask_warped;

  #ifdef DEBUG
    start = getTickCount();
  #endif

  for (int img_idx = 0; img_idx < numImage; img_idx++) {
      // Read image and resize it if necessary
      img = full_images[img_idx];

      Mat K, R;
      calibrations[img_idx].camera_matrix.convertTo(K, CV_32F);
      calibrations[img_idx].rectification.convertTo(R, CV_32F);
      // //Warping image based on precomputed spherical map
      // Rect image_roi = composedImageROI[img_idx];
      // img_warped.create(image_roi.height + 1, image_roi.width + 1, img.type());
      // remap(img, img_warped, composedImageUXMap[img_idx], composedImageUYMap[img_idx], INTER_LINEAR, BORDER_REFLECT);
      //
      // mask.create(img_size, CV_8U);
      // mask.setTo(Scalar::all(255));
      // //Warping mask based on precomputed spherical map
      // Rect mask_roi = composedMaskROI[img_idx];
      // mask_warped.create(mask_roi.height + 1, mask_roi.width + 1, mask.type());
      // remap(mask, mask_warped, composedMaskeUXMap[img_idx], composedMaskUYMap[img_idx], INTER_NEAREST, BORDER_CONSTANT);
      #ifdef DEBUG
        startWarp = getTickCount();
      #endif

      warper->warp(img, K, R, INTER_NEAREST, BORDER_CONSTANT, img_warped);

      mask.create(image_size, CV_8U);
      mask.setTo(Scalar::all(255));
      warper->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

      #ifdef DEBUG
        warpTime += (getTickCount() - startWarp) / getTickFrequency();
      #endif

      // Compensate exposure
      compensator->apply(img_idx, composedCorners[img_idx], img_warped, mask_warped);

      img_warped.convertTo(img_warped_s, CV_16S);
      img_warped.release();
      img.release();
      mask.release();

      dilate(masks_warped[img_idx], dilated_mask, Mat());
      resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);

      mask_warped = seam_mask & mask_warped;

      //build another canvas when the first image in the panorama is inputted
      if (img_idx == 0) {
          blender->prepare(composedCorners, updatedSizes);
      }

      // Blend the current image
      blender->feed(img_warped_s, mask_warped, composedCorners[img_idx]);
  }

  img_warped.release();
  img_warped_s.release();
  dilated_mask.release();
  seam_mask.release();
  mask.release();
  mask_warped.release();

  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Loop time: " << duration << "\n";
  #endif

  #ifdef DEBUG
    start = getTickCount();
  #endif

  Mat result_s, result_mask;
  blender->blend(result_s, result_mask);

  #ifdef DEBUG
    duration = (getTickCount() - start) / getTickFrequency();
    cout << "Blending time: " << duration << "\n";
    cout << "Warping 2nd time: " << warpTime << endl;
  #endif

  result_s.convertTo(result, CV_8U);
}
