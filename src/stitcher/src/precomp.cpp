#include "headers/precomp.hpp"

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

void buildComposedMaps() {
  for (int i = 0; i < numImage; ++i) {
      Mat_<float> K, R;
      calibrations[i].camera_matrix.convertTo(K, CV_32F);
      calibrations[i].rectification.convertTo(R, CV_32F);

      composedImageROI[i] = warper->buildMaps(image_size, K, R, composedImageUXMap[i], composedImageUYMap[i]);

      Mat mask;
      mask.create(image_size, CV_8U);
      mask.setTo(Scalar::all(255));

      //Warping mask based on precomputed spherical map
      UMat maskUXMap, maskUYMap;
      Rect mask_roi =  warper->buildMaps(image_size, K, R, maskUXMap, maskUYMap);
      composed_warped_masks[i].create(mask_roi.height + 1, mask_roi.width + 1, mask.type());
      remap(mask, composed_warped_masks[i], maskUXMap, maskUYMap, INTER_NEAREST, BORDER_CONSTANT);
  }
}

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

void initHelperTools() {
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
}

void precomp() {
  if (calibrations.empty()) {
    cout << "Calibration details have not been loaded yet" << endl;
    exit(ERROR);
  }

  initHelperTools();
  //Initialise image blender
  initBlender();

  //Build maps for warping images
  buildSphericalMaps();
  int64 start = clock();
  buildComposedMaps();
  cout << "composed maps: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
}
