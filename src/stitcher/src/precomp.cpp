#include "headers/precomp.hpp"

void buildComposedMaps() {
  for (int i = 0; i < numImage; ++i) {
      Mat_<float> K, R;
      calibrations[0].camera_matrix.convertTo(K, CV_32F);
      calibrations[startCamera - 1 + i].rectification.convertTo(R, CV_32F);

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

void initComposedCanvas() {
  // Update corners and sizes
  for (int i = 0; i < numImage; ++i) {
      // Update corner and size
      Mat K, R;
      calibrations[0].camera_matrix.convertTo(K, CV_32F);
      calibrations[startCamera - 1 + i].rectification.convertTo(R, CV_32F);
      Rect roi = warper->warpRoi(image_size, K, R);
      composedCorners[i] = roi.tl();
      updatedSizes[i] = roi.size();
  }

  //Preparing composition result
  dst_roi = resultRoi(composedCorners, updatedSizes);
  dst.create(dst_roi.size(), CV_16SC3);
  dst_mask.create(dst_roi.size(), CV_8U);
}

void initHelperTools() {
  double work_megapix = 1;
  double seam_megapix = 0.1;
  double work_scale = 0.5;
  double seam_scale = 1;
  double seam_work_aspect = 1;
  float warped_image_scale;

  work_scale = min(1.0, sqrt(work_megapix * 1e6 / image_size.area()));
  seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / image_size.area()));
  seam_work_aspect = seam_scale / work_scale;

  //TODO: Using constant K
  warped_image_scale = calibrations[0].camera_matrix.at<double>(0, 0);

  // Warp images and their masks
  warper_creator = makePtr<cv::SphericalWarper>();
  if (!warper_creator) {
      cout << "Can't create cylindrical warper" << endl;
      exit(ERROR);
  }
  warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
}

void precomp() {
  if (calibrations.empty()) {
    cout << "Calibration details have not been loaded yet" << endl;
    exit(ERROR);
  }

  initHelperTools();
  buildComposedMaps();
  initComposedCanvas();
  dst_weight_map.create(dst_roi.size(), CV_32F);

  //Build weight map that is used in feather blending
  float sharpness = 0.02f;
  for (int i = 0; i < numImage; i++) {
    createWeightMap(composed_warped_masks[i], sharpness, weight_maps[i]);
  }

  warper.release();
  warper_creator.release();
}
