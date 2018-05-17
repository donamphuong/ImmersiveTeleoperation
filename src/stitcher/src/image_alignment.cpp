#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "details.cpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

double work_scale = 0.5;
double seam_scale = 1;
double compose_scale = 1;
double compose_work_aspect = 1;
double seam_work_aspect = 1;
float warped_image_scale;
Size image_scale_size = Size(422, 237);
Size image_size = Size(1920, 1080);

vector<UMat> sphericalImageUXMap(numImage);
vector<UMat> sphericalImageUYMap(numImage);
vector<Rect> sphericalImageROI(numImage);
vector<UMat> sphericalMaskUXMap(numImage);
vector<UMat> sphericalMaskUYMap(numImage);
vector<Rect> sphericalMaskROI(numImage);

vector<UMat> composedImageUXMap(numImage);
vector<UMat> composedImageUYMap(numImage);
vector<Rect> composedImageROI(numImage);
vector<UMat> composedMaskeUXMap(numImage);
vector<UMat> composedMaskUYMap(numImage);
vector<Rect> composedMaskROI(numImage);

Ptr<FeaturesFinder> finder;
Ptr<WarperCreator> warper_creator;
Ptr<RotationWarper> warper;
Ptr<ExposureCompensator> compensator;
Ptr<SeamFinder> seam_finder;
Ptr<Blender> blender;

void buildComposedMaps() {
  // Compute relative scales
  compose_work_aspect = compose_scale / work_scale;

  // Update warped image scale
  warped_image_scale *= static_cast<float>(compose_work_aspect);
  warper = warper_creator->create(warped_image_scale);

  for (int i = 0; i < numImage; ++i) {
      Mat_<float> K, R;
      newCameraMatrix.convertTo(K, CV_32F);
      rotationMatrix[i].convertTo(R, CV_32F);

      composedImageROI[i] = warper->buildMaps(image_size, K, R, composedImageUXMap[i], composedImageUYMap[i]);
      composedMaskROI[i] = warper->buildMaps(image_size, K, R, composedMaskeUXMap[i], composedMaskUYMap[i]);
  }
}

void buildSphericalMaps() {

  // Warp images and their masks
  warper_creator = makePtr<cv::SphericalWarperGpu>();

  if (!warper_creator) {
      cout << "Can't create cylindrical warper" << endl;
      exit(1);
  }
  warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

  for (int i = 0; i < numImage; ++i) {
      Mat_<float> K, R;
      newCameraMatrix.convertTo(K, CV_32F);
      rotationMatrix[i].convertTo(R, CV_32F);
      float swa = (float)seam_work_aspect;
      K(0,0) *= swa; K(0,2) *= swa;
      K(1,1) *= swa; K(1,2) *= swa;

      sphericalImageROI[i] = warper->buildMaps(image_scale_size, K, R, sphericalImageUXMap[i], sphericalImageUYMap[i]);
      sphericalMaskROI[i] = warper->buildMaps(image_scale_size, K, R, sphericalMaskUXMap[i], sphericalMaskUYMap[i]);
  }
}

void precomp() {
  double work_megapix = 0.6;
  double seam_megapix = 0.1;

  work_scale = min(1.0, sqrt(work_megapix * 1e6 / image_size.area()));
  seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / image_size.area()));
  seam_work_aspect = seam_scale / work_scale;

  //TODO: Using constant K
  warped_image_scale = newCameraMatrix.at<double>(0, 0);

  buildSphericalMaps();
  // buildComposedMaps();
}

int beforeStitch(vector<string> img_names) {
    Mat full_img, img;
    int num_images = static_cast<int>(img_names.size());
    vector<ImageFeatures> features(num_images);
    vector<Mat> images(num_images);
    vector<Size> full_img_sizes(num_images);
    Ptr<FeaturesFinder> finder = makePtr<SurfFeaturesFinderGpu>();
    double seam_work_aspect = 1;

    for (int i = 0; i < num_images; ++i)
    {
      full_img = imread(img_names[i]);
      full_img_sizes[i] = full_img.size();

      if (full_img.empty())
      {
        cout << "Cannot open images" << endl;
         return -1;
      }
      resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
      (*finder)(img, features[i]);
      features[i].img_idx = i;

      resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
    }

  float conf_thresh = 1.f;
  vector<MatchesInfo> pairwise_matches;
  float match_conf = 0.3f;

    Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestMatcher>(true, match_conf);

    (*matcher)(features, pairwise_matches);
    matcher->collectGarbage();

    // // Leave only images we are sure are from the same panorama
    // vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
    // vector<Mat> img_subset;
    // vector<String> img_names_subset;
    // vector<Size> full_img_sizes_subset;
    // for (size_t i = 0; i < indices.size(); ++i)
    // {
    //     img_names_subset.push_back(img_names[indices[i]]);
    //     img_subset.push_back(images[indices[i]]);
    //     full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
    // }
    //
    // images = img_subset;
    // img_names = img_names_subset;
    // full_img_sizes = full_img_sizes_subset;

    // Check if we still have enough images
    num_images = static_cast<int>(img_names.size());
    if (num_images < 2)
    {
        cout << "Need more images" << endl;
        return -1;
    }

    Ptr<Estimator> estimator = makePtr<HomographyBasedEstimator>();

    vector<CameraParams> cameras;
    if (!(*estimator)(features, pairwise_matches, cameras))
    {
        cout << "Homography estimation failed.\n";
        return -1;
    }

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;

        cout << "K" << cameras[i].K() << endl;
        cout << "R" << cameras[i].R << endl;

    }
}

void cudaResize(Mat src, Mat &dst, Size size, double fx, double fy) {
  clock_t start = clock();
  double duration;
  cuda::GpuMat inputGpu(src);
  cuda::GpuMat outputGpu;
  duration += (clock() - start) / (double) CLOCKS_PER_SEC;

  cuda::resize(inputGpu, outputGpu, size, fx, fy, INTER_LINEAR);
  outputGpu.download(dst);

  start = clock();
  inputGpu.release();
  outputGpu.release();
  duration += (clock() - start) / (double) CLOCKS_PER_SEC;
  // cout << "resizing " << duration << endl;
}

void stitch(vector<Mat> full_images) {
    clock_t start, startWarp;
    double duration, warpTime;
    Mat K;
    newCameraMatrix.convertTo(K, CV_32F);

    Mat full_img, img;
    vector<Mat> images(numImage);
    vector<Size> full_img_sizes(numImage);

    start = clock();
    for (int i = 0; i < numImage; ++i)
    {
        full_img = full_images[i];
        full_img_sizes[i] = full_img.size();

        if (full_img.empty())
        {

            cout << "Image is empty" << endl;
            exit(-1);
        }

        resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
        resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);

        images[i] = img.clone();
    }
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Reading images " << duration << endl;

    full_img.release();
    img.release();

    vector<UMat> masks(numImage);
    vector<Point> corners(numImage);
    vector<Size> sizes(numImage);
    vector<UMat> masks_warped(numImage);
    vector<UMat> images_warped(numImage);

    // Preapre images masks
    for (int i = 0; i < numImage; ++i) {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }

    start = clock();
    //Warping Image
    for (int i = 0; i < numImage; ++i) {
      //Warping image based on precomputed spherical map
      Rect image_roi = sphericalImageROI[i];
      images_warped[i].create(image_roi.height + 1, image_roi.width + 1, images[i].type());
      remap(images[i], images_warped[i], sphericalImageUXMap[i], sphericalImageUYMap[i], INTER_LINEAR, BORDER_REFLECT);
      corners[i] = image_roi.tl();
      sizes[i] = images_warped[i].size();

      //Warping mask based on precomputed spherical map
      Rect mask_roi = sphericalMaskROI[i];
      masks_warped[i].create(mask_roi.height + 1, mask_roi.width + 1, masks[i].type());
      remap(masks[i], masks_warped[i], sphericalMaskUXMap[i], sphericalMaskUYMap[i], INTER_NEAREST, BORDER_CONSTANT);
    }

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Warping time: " << duration << "\n";

    vector<UMat> images_warped_f(numImage);
    for (int i = 0; i < numImage; ++i) {
      images_warped[i].convertTo(images_warped_f[i], CV_32F);
    }

    compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN);
    start = clock();
    compensator->feed(corners, images_warped, masks_warped);
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Exposure Compensating Time: " << duration << endl;

    seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
    if (!seam_finder) {
        cout << "Can't create graph cut seam finder" << endl;
        exit(1);
    }

    start = clock();
    seam_finder->find(images_warped_f, corners, masks_warped);
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Finding seam time: " << duration << "\n";

    // Release unused memory
    images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    bool has_updated_corners_sizes = false;

    start = clock();
    for (int img_idx = 0; img_idx < numImage; ++img_idx) {
        // Read image and resize it if necessary
        img = full_images[img_idx];
        Size img_size = img.size();

        if (!has_updated_corners_sizes) {
          // Update corners and sizes
          for (int i = 0; i < numImage; ++i) {
              // Update corner and size
              Mat R;
              rotationMatrix[i].convertTo(R, CV_32F);
              Rect roi = warper->warpRoi(img_size, K, R);
              corners[i] = roi.tl();
              sizes[i] = roi.size();
          }
          has_updated_corners_sizes = true;
        }

        Mat R;
        rotationMatrix[img_idx].convertTo(R, CV_32F);

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
        warper->warp(img, K, R, INTER_NEAREST, BORDER_CONSTANT, img_warped);

        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

        // Compensate exposure
        compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();

        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);

        mask_warped = seam_mask & mask_warped;

        startWarp = clock();
        if (!blender) {
            float blend_strength = 5;
            blender = Blender::createDefault(Blender::MULTI_BAND, false);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;

            MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
            mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));

            start = clock();
            blender->prepare(corners, sizes);
            duration += (clock() - start) / (double) CLOCKS_PER_SEC;
            cout << "Preparing Blender: " << duration << endl;
        }
        warpTime += (clock() - startWarp) / (double) CLOCKS_PER_SEC;

        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    duration += (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Loop time: " << duration << "\n";

    start = clock();
    Mat result, result_s, result_mask;
    blender->blend(result_s, result_mask);
    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Blending time: " << duration << "\n";
    cout << "Resizing time: " << warpTime << endl;

    // result_s.convertTo(result, CV_8U);
    // namedWindow("warped", WINDOW_NORMAL);
    // resizeWindow("warped", 1024, 600);
    // imshow ("warped", result);
    // waitKey();
}

int main(int argc, char** argv) {
  clock_t start;
  double duration;
  start = clock();
  getCalibrationDetails();
  duration = (clock() - start) / (double) CLOCKS_PER_SEC;
  cout << "Get Calibration Details time: " << duration << "\n";

  vector<Mat> images;

  for (int i = numImage; i > 0; i--) {
    string filename = "test" + to_string(i) + ".png";
    images.push_back(imread(filename));
  }

  precomp();
  start = clock();
  stitch(images);
  duration = (clock() - start) / (double) CLOCKS_PER_SEC;
  cout << "printf: " << duration << "\n";

  return 0;
}
