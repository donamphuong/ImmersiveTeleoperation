#include "headers/stitch.hpp"

class Read : public cv::ParallelLoopBody {
  private:
    vector<Mat> full_images;
    vector<Mat> &images;
  public:
    Read(vector<Mat> inputFullImages, vector<Mat> &inputImages)
        :full_images(inputFullImages), images(inputImages) {}

    virtual void operator()(const cv::Range& range) const {
      Mat full_img, img;
      for(int i = range.start; i < range.end; i++) {
          /* divide image in 'diff' number
          of parts and process simultaneously */

          full_img = full_images[i];

          // resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
          resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);

          images[i] = img.clone();
      }
    }
};

class Warp : public cv::ParallelLoopBody {
  private:
    vector<Point> &corners;
    vector<Size> &sizes;
    vector<UMat> &images_warped;
    vector<Mat> images;
  public:
    Warp(vector<Point> &inputCorners, vector<Size> &inputSizes, vector<UMat> &inputImagesWarped, vector<Mat> inputImages)
        :corners(inputCorners), sizes(inputSizes), images_warped(inputImagesWarped), images(inputImages) {}

    virtual void operator()(const cv::Range& range) const {
      for(int i = range.start; i < range.end; i++) {
        Rect image_roi = sphericalImageROI[i];
        images_warped[i].create(image_roi.height + 1, image_roi.width + 1, images[i].type());
        remap(images[i], images_warped[i], sphericalImageUXMap[i], sphericalImageUYMap[i], INTER_LINEAR, BORDER_REFLECT);

        corners[i] = image_roi.tl();
        sizes[i] = images_warped[i].size();
      }
    }
};

class Compose : public cv::ParallelLoopBody {
  private:
    vector<Mat> full_images;
    vector<Mat> &images_warped_s;
    vector<Mat> &warped_masks;
  public:
    Compose(vector<Mat> inputFullImages, vector<Mat> &inputImagesWarped,
            vector<Mat> &inputWarpedMasks)
        :full_images(inputFullImages), images_warped_s(inputImagesWarped),
        warped_masks(inputWarpedMasks){}

    virtual void operator()(const cv::Range& range) const {
      for (int img_idx = range.start; img_idx < range.end; img_idx++) {
        Mat img, img_warped;
        Mat dilated_mask, seam_mask, mask;
        // Read image and resize it if necessary
        img = full_images[img_idx];

        // #ifdef DEBUG
        //   startWarp = getTickCount();
        // #endif

        Mat K, R;
        calibrations[img_idx].camera_matrix.convertTo(K, CV_32F);
        calibrations[img_idx].rectification.convertTo(R, CV_32F);
        // #ifdef DEBUG
        //   warpTime += (getTickCount() - startWarp) / getTickFrequency();
        // #endif

        //Warping image based on precomputed spherical map
        Rect image_roi = composedImageROI[img_idx];
        img_warped.create(image_roi.height + 1, image_roi.width + 1, img.type());
        remap(img, img_warped, composedImageUXMap[img_idx], composedImageUYMap[img_idx], INTER_LINEAR, BORDER_REFLECT);

        mask.create(image_size, CV_8U);
        mask.setTo(Scalar::all(255));
        //Warping mask based on precomputed spherical map
        Rect mask_roi = composedMaskROI[img_idx];
        warped_masks[img_idx].create(mask_roi.height + 1, mask_roi.width + 1, mask.type());
        remap(mask, warped_masks[img_idx], composedMaskUXMap[img_idx], composedMaskUYMap[img_idx], INTER_NEAREST, BORDER_CONSTANT);

        img_warped.convertTo(images_warped_s[img_idx], CV_16S);

        // Compensate exposure
        compensator->apply(img_idx, composedCorners[img_idx], img_warped, warped_masks[img_idx]);

        dilate(warped_masks[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, warped_masks[img_idx].size(), 0, 0, INTER_LINEAR_EXACT);

        warped_masks[img_idx] = seam_mask & warped_masks[img_idx];

        // Blend the current image
        blender->feed(images_warped_s[img_idx], warped_masks[img_idx], composedCorners[img_idx]);
      }
    }
};

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
  // // Compute relative scales
  // compose_work_aspect = compose_scale / work_scale;
  //
  // // Update warped image scale
  // warped_image_scale *= static_cast<float>(compose_work_aspect);
  // warper = warper_creator->create(warped_image_scale);

  for (int i = 0; i < numImage; ++i) {
      Mat_<float> K, R;
      calibrations[i].camera_matrix.convertTo(K, CV_32F);
      calibrations[i].rectification.convertTo(R, CV_32F);

      composedImageROI[i] = warper->buildMaps(image_size, K, R, composedImageUXMap[i], composedImageUYMap[i]);
      composedMaskROI[i] = warper->buildMaps(image_size, K, R, composedMaskUXMap[i], composedMaskUYMap[i]);
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
  buildComposedMaps();
}

void compose(vector<Mat> full_images, Mat &result) {
  #ifdef DEBUG
    int64 start, startWarp;
    double duration, warpTime;
  #endif

  vector<Mat> images_warped_s(numImage);
  vector<Mat> warped_masks(numImage);

  #ifdef DEBUG
    start = getTickCount();
  #endif

  //build another canvas when the first image in the panorama is inputted
  blender->prepare(composedCorners, updatedSizes);

  parallel_for_(Range(0, numImage), Compose(full_images, images_warped_s, warped_masks));

  warped_masks.clear();
  images_warped_s.clear();

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
    cout << "Feeding time: " << warpTime << "\n";
  #endif

  result_s.convertTo(result, CV_8U);
}

Mat stitch(const vector<Mat> full_images, Mat &result) {
  #ifdef DEBUG
    cout << endl << "START" << endl;
    int64 start, startWarp;
    double duration, warpTime;
  #endif

  Mat full_img, img;
  vector<Mat> images(numImage);

  #ifdef DEBUG
    start = getTickCount();
  #endif

  // for (int i = 0; i < numImage; ++i) {
  //   full_img = full_images[i];
  //
  //   resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
  //
  //   images[i] = img.clone();
  // }
  parallel_for_(Range(0, numImage), Read(full_images, images));

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
  // // Warping image based on precomputed spherical map
  // for (int i = 0; i < numImage; ++i) {
  //   Rect image_roi = sphericalImageROI[i];
  //   images_warped[i].create(image_roi.height + 1, image_roi.width + 1, images[i].type());
  //   remap(images[i], images_warped[i], sphericalImageUXMap[i], sphericalImageUYMap[i], INTER_LINEAR, BORDER_REFLECT);
  //
  //   corners[i] = image_roi.tl();
  //   sizes[i] = images_warped[i].size();
  // }

  parallel_for_(Range(0, numImage), Warp(corners, sizes, images_warped, images));
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
  masks_warped.clear();

  compose(full_images, result);
}
