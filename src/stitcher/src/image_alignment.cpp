#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "details.cpp"
#include "opencv2/stitching/detail/timelapsers.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

/*void stitch(vector<Mat> images) {
  Mat stitched(images[0].rows, images[0].cols + images[1].cols, images[0].type());

  //stores top left corners coordinates of each image
  vector<UMat> imagesWarped(numImage);
  vector<UMat> masks(numImage);
  vector<UMat> masksWarped(numImage);
  vector<Point> corners(numImage);
  vector<Size> sizes(numImage);

  Ptr<WarperCreator> warperCreator = makePtr<cv::CylindricalWarperGpu>();
  Ptr<RotationWarper> warper = warperCreator->create(static_cast<float>(1000));

  // the mask of an image is a white rectangle of the same size as the image
  for (int i = 0; i < numImage; i++) {
    masks[i].create(images[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));

    Mat K, R;
    calibrations[i].camera_matrix.convertTo(K, CV_32F);
    calibrations[i].rectification.convertTo(R, CV_32F);

    corners[i] = warper->warp(images[i], K, R, INTER_LINEAR, BORDER_REFLECT, imagesWarped[i]);
    cout << corners[i] << endl;
    warper->warp(masks[i], K, R, INTER_NEAREST, BORDER_CONSTANT, masksWarped[i]);

    sizes[i] = imagesWarped[i].size();
  }

  Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN_BLOCKS);
  compensator->feed(corners, imagesWarped, masks);

  vector<UMat> imagesWarped_F(numImage);
  for (int i = 0; i < numImage; i++) {
    imagesWarped[i].convertTo(imagesWarped_F[i], CV_32F);
  }
  Ptr<SeamFinder> seamFinder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
  seamFinder->find(imagesWarped_F, corners, masks);


  // images.clear();
  imagesWarped.clear();
  imagesWarped_F.clear();
  masks.clear();

  Ptr<Blender> blender;

  Mat im, imWarped, fullImage, imWarpedS, seamMask, dilatedMask, mask, maskWarped;
  double composeMegapix = -1, composeScale = 1, composeWorkAspect = 1;
  bool isComposeScaleSet = false;

  for(int i = 0; i < numImage; i++) {
    fullImage = images[i];
    if (!isComposeScaleSet) {
      if (composeMegapix > 0) {
        composeScale = min(1.0, sqrt(composeMegapix * 1e6 / fullImage.size().area()));
      }
      isComposeScaleSet = true;

      //Compute relative scale
      composeWorkAspect = composeScale/workScale;
    }

    Mat K, R;
    calibrations[i].camera_matrix.convertTo(K, CV_32F);
    calibrations[i].rectification.convertTo(R, CV_32F);
    warper->warp(im, K, R, INTER_LINEAR, BORDER_CONSTANT, imWarped);

    mask.create(imSize, CV_8U);
    mask.setTo(Scalar::all(255));
    warper->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, maskWarped);

    //Compensate exposure
    compensator->apply(i, corners[i], imWarped, maskWarped);

    imWarped.convertTo(imWarpedS, CV_16S);
    imWarped.release();
    im.release();
    mask.release();

    dilate(masksWarped[i], dilatedMask, Mat());
    resize(dilatedMask, seamMask, maskWarped.size(), 0, 0, INTER_LINEAR_EXACT);
    maskWarped = seamMask & maskWarped;

    if (!blender) {
      blender = Blender::createDefault(Blender::MULTI_BAND, true);
      blender->prepare(corners, sizes);
    }

    blender->feed(imWarpedS, maskWarped, corners[i]);
  }

  Mat result, result_s, result_mask;
  blender->blend(result_s, result_mask);

  result_s.convertTo(result, CV_8U);
  namedWindow("warped", WINDOW_NORMAL);
  resizeWindow("warped", 1024, 600);
  imshow ("warped", result);
  waitKey();

}*/

vector<String> img_names;
bool preview = false;
bool try_cuda = false;
double work_megapix = 0.6;
double seam_megapix = 0.1;
double compose_megapix = -1;
float conf_thresh = 1.f;
string features_type = "surf";
string matcher_type = "homography";
string estimator_type = "homography";
string ba_cost_func = "ray";
string ba_refine_mask = "xxxxx";
bool do_wave_correct = true;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
bool save_graph = false;
std::string save_graph_to;
string warp_type = "spherical";
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
float match_conf = 0.3f;
string seam_find_type = "gc_color";
int blend_type = Blender::MULTI_BAND;
int timelapse_type = Timelapser::AS_IS;
float blend_strength = 5;
string result_name = "result.jpg";
bool timelapse = false;
int range_width = -1;

void stitch(vector<string> img_names) {
    Ptr<FeaturesFinder> finder = makePtr<SurfFeaturesFinderGpu>();
    double work_scale = 1, seam_scale = 1, compose_scale = 1;
    bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

    Mat full_img, img;
    vector<Mat> images(numImage);
    vector<Size> full_img_sizes(numImage);
    double seam_work_aspect = 1;

    for (int i = 0; i < numImage; ++i)
    {
        full_img = imread(img_names[i]);
        full_img_sizes[i] = full_img.size();

        if (full_img.empty())
        {
            cout << "Can't open image " << img_names[i];
            exit(-1);
        }
        if (work_megapix < 0)
        {
            img = full_img;
            work_scale = 1;
            is_work_scale_set = true;
        }
        else
        {
            if (!is_work_scale_set)
            {
                work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }

        resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
        images[i] = img.clone();
    }

    full_img.release();
    img.release();

    // Ptr<detail::BundleAdjusterBase> adjuster;
    // if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
    // else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
    // else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
    // else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
    // else
    // {
    //     cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
    //     return -1;
    // }
    // adjuster->setConfThresh(conf_thresh);
    // Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    // if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    // if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    // if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    // if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    // if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    // adjuster->setRefinementMask(refine_mask);
    // if (!(*adjuster)(features, pairwise_matches, cameras))
    // {
    //     cout << "Camera parameters adjusting failed.\n";
    //     return -1;
    // }

    // Find median focal length
    vector<double> focals;
    for (size_t i = 0; i < calibrations.size(); ++i)
    {
        // LOGLN("Camera #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
        focals.push_back((calibrations[i].camera_matrix.at<double>(0, 0) + calibrations[i].camera_matrix.at<double>(1, 1))/2);
    }

    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

    // if (do_wave_correct)
    // {
    //     vector<Mat> rmats;
    //     for (size_t i = 0; i < calibrations.size(); ++i)
    //         rmats.push_back(calibrations[i].rectification.clone());
    //     waveCorrect(rmats, wave_correct);
    //     for (size_t i = 0; i < calibrations.size(); ++i)
    //         calibrations[i].rectification = rmats[i];
    // }

    vector<Point> corners(numImage);
    vector<UMat> masks_warped(numImage);
    vector<UMat> images_warped(numImage);
    vector<Size> sizes(numImage);
    vector<UMat> masks(numImage);

    // Preapre images masks
    for (int i = 0; i < numImage; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }

    // Warp images and their masks

    Ptr<WarperCreator> warper_creator;
    warper_creator = makePtr<cv::CylindricalWarperGpu>();

    cout << "HELLO" << endl;
    if (!warper_creator)
    {
        cout << "Can't create the following warper '" << warp_type << "'\n";
        exit(1);
    }

    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

    for (int i = 0; i < numImage; ++i)
    {
        Mat_<float> K, R;
        calibrations[i].camera_matrix.convertTo(K, CV_32F);
        calibrations[i].rectification.convertTo(R, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;

        corners[i] = warper->warp(images[i], K, R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();

        warper->warp(masks[i], K, R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }

    vector<UMat> images_warped_f(numImage);
    for (int i = 0; i < numImage; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);

    // LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
    compensator->feed(corners, images_warped, masks_warped);

    Ptr<SeamFinder> seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
    if (!seam_finder)
    {
        cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
        exit(1);
    }

    seam_finder->find(images_warped_f, corners, masks_warped);

    // Release unused memory
    images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Ptr<Timelapser> timelapser;
    //double compose_seam_aspect = 1;
    double compose_work_aspect = 1;

    for (int img_idx = 0; img_idx < numImage; ++img_idx)
    {
        // LOGLN("Compositing image #" << indices[img_idx]+1);

        // Read image and resize it if necessary
        full_img = imread(img_names[img_idx]);
        if (!is_compose_scale_set)
        {
            if (compose_megapix > 0)
                compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
            is_compose_scale_set = true;

            // Compute relative scales
            //compose_seam_aspect = compose_scale / seam_scale;
            compose_work_aspect = compose_scale / work_scale;

            // Update warped image scale
            warped_image_scale *= static_cast<float>(compose_work_aspect);
            warper = warper_creator->create(warped_image_scale);

            // Update corners and sizes
            for (int i = 0; i < numImage; ++i)
            {
                // Update intrinsics
                // cameras[i].focal *= compose_work_aspect;
                // cameras[i].ppx *= compose_work_aspect;
                // cameras[i].ppy *= compose_work_aspect;

                // Update corner and size
                Size sz = full_img_sizes[i];
                if (std::abs(compose_scale - 1) > 1e-1)
                {
                    sz.width = cvRound(full_img_sizes[i].width * compose_scale);
                    sz.height = cvRound(full_img_sizes[i].height * compose_scale);
                }

                Mat K, R;
                calibrations[i].camera_matrix.convertTo(K, CV_32F);
                calibrations[i].rectification.convertTo(R, CV_32F);
                Rect roi = warper->warpRoi(sz, K, R);
                corners[i] = roi.tl();
                sizes[i] = roi.size();
            }
        }
        if (abs(compose_scale - 1) > 1e-1)
            resize(full_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = full_img;
        full_img.release();
        Size img_size = img.size();

        Mat K, R;
        calibrations[img_idx].camera_matrix.convertTo(K, CV_32F);
        calibrations[img_idx].rectification.convertTo(R, CV_32F);

        // Warp the current image
        warper->warp(img, K, R, INTER_LINEAR, BORDER_REFLECT, img_warped);

        // Warp the current image mask
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

        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(Blender::MULTI_BAND, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;

            MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
            mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            // LOGLN("Multi-band blender, number of bands: " << mb->numBands());

            blender->prepare(corners, sizes);
        }
        else if (!timelapser && timelapse)
        {
            timelapser = Timelapser::createDefault(timelapse_type);
            timelapser->initialize(corners, sizes);
        }

        // Blend the current image
        if (timelapse)
        {
            timelapser->process(img_warped_s, Mat::ones(img_warped_s.size(), CV_8UC1), corners[img_idx]);
            String fixedFileName;
            size_t pos_s = String(img_names[img_idx]).find_last_of("/\\");
            if (pos_s == String::npos)
            {
                fixedFileName = "fixed_" + img_names[img_idx];
            }
            else
            {
                fixedFileName = "fixed_" + String(img_names[img_idx]).substr(pos_s + 1, String(img_names[img_idx]).length() - pos_s);
            }
            imwrite(fixedFileName, timelapser->getDst());
        }
        else
        {
            blender->feed(img_warped_s, mask_warped, corners[img_idx]);
        }
    }

    if (!timelapse)
    {
        Mat result, result_mask;
        blender->blend(result, result_mask);

        // LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

        imwrite(result_name, result);
    }

    // LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
    exit(0);
}

int findCameraPose() {
  // Check if have enough images
    int num_img = static_cast<int>(img_names.size());
  double work_scale = 1, seam_scale = 1, compose_scale = 1;
  bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

   Ptr<FeaturesFinder> finder = makePtr<SurfFeaturesFinderGpu>();
   Mat full_img, img;
   vector<ImageFeatures> features(num_img);
   vector<Mat> images(num_img);
   vector<Size> full_img_sizes(num_img);
   double seam_work_aspect = 1;

   for (int i = 0; i < num_img; ++i)
   {
       full_img = imread(img_names[i]);
       full_img_sizes[i] = full_img.size();

       if (full_img.empty())
       {
           // LOGLN("Can't open image " << img_names[i]);
           return -1;
       }
       if (work_megapix < 0)
       {
           img = full_img;
           work_scale = 1;
           is_work_scale_set = true;
       }
       else
       {
           if (!is_work_scale_set)
           {
               work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
               is_work_scale_set = true;
           }
           resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
       }
       if (!is_seam_scale_set)
       {
           seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
           seam_work_aspect = seam_scale / work_scale;
           is_seam_scale_set = true;
       }

       (*finder)(img, features[i]);
       features[i].img_idx = i;
       // LOGLN("Features in image #" << i+1 << ": " << features[i].keypoints.size());

       resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
       images[i] = img.clone();
   }

   finder->collectGarbage();
   full_img.release();
   img.release();

   // LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

   // LOG("Pairwise matching");
   vector<MatchesInfo> pairwise_matches;
   Ptr<FeaturesMatcher> matcher;
   if (matcher_type == "affine")
       matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
   else if (range_width==-1)
       matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
   else
       matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);

   (*matcher)(features, pairwise_matches);
   matcher->collectGarbage();

   // LOGLN("Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

   // Leave only images we are sure are from the same panorama
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
   num_img = static_cast<int>(img_names.size());
   if (num_img < 2)
   {
      cout << "Need more images" << endl;
       // LOGLN("Need more images");
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
       // LOGLN("Initial camera intrinsics #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
   }

   Ptr<detail::BundleAdjusterBase> adjuster;
   if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
   else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
   else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
   else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
   else
   {
       cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
       return -1;
   }
   adjuster->setConfThresh(conf_thresh);
   Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
   if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
   if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
   if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
   if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
   if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
   adjuster->setRefinementMask(refine_mask);
   if (!(*adjuster)(features, pairwise_matches, cameras))
   {
       cout << "Camera parameters adjusting failed.\n";
       return -1;
   }

   // Find median focal length

   vector<double> focals;
   for (size_t i = 0; i < cameras.size(); ++i)
   {
       // LOGLN("Camera #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
       focals.push_back(cameras[i].focal);
   }

   sort(focals.begin(), focals.end());
   float warped_image_scale;
   if (focals.size() % 2 == 1)
       warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
   else
       warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

   if (do_wave_correct)
   {
       vector<Mat> rmats;
       for (size_t i = 0; i < cameras.size(); ++i)
           rmats.push_back(cameras[i].R.clone());
       waveCorrect(rmats, wave_correct);
       for (size_t i = 0; i < cameras.size(); ++i)
           cameras[i].R = rmats[i];
   }

   for (int i = 0; i < num_img; i++) {
     cout << std::to_string(i) << endl;
     cout << "R" << endl << cameras[i].R << endl;
     cout << "K" << endl << cameras[i].K() << endl;
   }


   // LOGLN("Warping images (auxiliary)... ");

   vector<Point> corners(num_img);
   vector<UMat> masks_warped(num_img);
   vector<UMat> images_warped(num_img);
   vector<Size> sizes(num_img);
   vector<UMat> masks(num_img);

   // Preapre images masks
   for (int i = 0; i < num_img; ++i)
   {
       masks[i].create(images[i].size(), CV_8U);
       masks[i].setTo(Scalar::all(255));
   }

   // Warp images and their masks

   Ptr<WarperCreator> warper_creator = makePtr<cv::CylindricalWarperGpu>();

   if (!warper_creator)
   {
       cout << "Can't create the following warper '" << warp_type << "'\n";
       return 1;
   }

   Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

   for (int i = 0; i < num_img; ++i)
   {
       Mat_<float> K;
       cameras[i].K().convertTo(K, CV_32F);

       float swa = (float)seam_work_aspect;
       K(0,0) *= swa; K(0,2) *= swa;
       K(1,1) *= swa; K(1,2) *= swa;

       corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
       sizes[i] = images_warped[i].size();

       warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
   }

   vector<UMat> images_warped_f(num_img);
   for (int i = 0; i < num_img; ++i)
       images_warped[i].convertTo(images_warped_f[i], CV_32F);

   // LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

   Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
   compensator->feed(corners, images_warped, masks_warped);

   Ptr<SeamFinder> seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
   if (!seam_finder)
   {
       cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
       return 1;
   }

   seam_finder->find(images_warped_f, corners, masks_warped);

   // Release unused memory
   images.clear();
   images_warped.clear();
   images_warped_f.clear();
   masks.clear();

   // LOGLN("Compositing...");

   Mat img_warped, img_warped_s;
   Mat dilated_mask, seam_mask, mask, mask_warped;
   Ptr<Blender> blender;
   Ptr<Timelapser> timelapser;
   //double compose_seam_aspect = 1;
   double compose_work_aspect = 1;

   for (int img_idx = 0; img_idx < num_img; ++img_idx)
   {
       // LOGLN("Compositing image #" << indices[img_idx]+1);

       // Read image and resize it if necessary
       full_img = imread(img_names[img_idx]);
       if (!is_compose_scale_set)
       {
           if (compose_megapix > 0)
               compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
           is_compose_scale_set = true;

           // Compute relative scales
           //compose_seam_aspect = compose_scale / seam_scale;
           compose_work_aspect = compose_scale / work_scale;

           // Update warped image scale
           warped_image_scale *= static_cast<float>(compose_work_aspect);
           warper = warper_creator->create(warped_image_scale);

           // Update corners and sizes
           for (int i = 0; i < num_img; ++i)
           {
               // Update intrinsics
               cameras[i].focal *= compose_work_aspect;
               cameras[i].ppx *= compose_work_aspect;
               cameras[i].ppy *= compose_work_aspect;

               // Update corner and size
               Size sz = full_img_sizes[i];
               if (std::abs(compose_scale - 1) > 1e-1)
               {
                   sz.width = cvRound(full_img_sizes[i].width * compose_scale);
                   sz.height = cvRound(full_img_sizes[i].height * compose_scale);
               }

               Mat K;
               cameras[i].K().convertTo(K, CV_32F);
               Rect roi = warper->warpRoi(sz, K, cameras[i].R);
               corners[i] = roi.tl();
               sizes[i] = roi.size();
           }
       }
       if (abs(compose_scale - 1) > 1e-1)
           resize(full_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
       else
           img = full_img;
       full_img.release();
       Size img_size = img.size();

       Mat K;
       cameras[img_idx].K().convertTo(K, CV_32F);

       // Warp the current image
       warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

       // Warp the current image mask
       mask.create(img_size, CV_8U);
       mask.setTo(Scalar::all(255));
       warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

       // Compensate exposure
       compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

       img_warped.convertTo(img_warped_s, CV_16S);
       img_warped.release();
       img.release();
       mask.release();

       dilate(masks_warped[img_idx], dilated_mask, Mat());
       resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
       mask_warped = seam_mask & mask_warped;

       if (!blender && !timelapse)
       {
           blender = Blender::createDefault(blend_type, try_cuda);
           Size dst_sz = resultRoi(corners, sizes).size();
           float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
           if (blend_width < 1.f)
               blender = Blender::createDefault(Blender::NO, try_cuda);
           else if (blend_type == Blender::MULTI_BAND)
           {
               MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
               mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
               // LOGLN("Multi-band blender, number of bands: " << mb->numBands());
           }
           else if (blend_type == Blender::FEATHER)
           {
               FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
               fb->setSharpness(1.f/blend_width);
               // LOGLN("Feather blender, sharpness: " << fb->sharpness());
           }
           blender->prepare(corners, sizes);
       }
       else if (!timelapser && timelapse)
       {
           timelapser = Timelapser::createDefault(timelapse_type);
           timelapser->initialize(corners, sizes);
       }

       // Blend the current image
       if (timelapse)
       {
           timelapser->process(img_warped_s, Mat::ones(img_warped_s.size(), CV_8UC1), corners[img_idx]);
           String fixedFileName;
           size_t pos_s = String(img_names[img_idx]).find_last_of("/\\");
           if (pos_s == String::npos)
           {
               fixedFileName = "fixed_" + img_names[img_idx];
           }
           else
           {
               fixedFileName = "fixed_" + String(img_names[img_idx]).substr(pos_s + 1, String(img_names[img_idx]).length() - pos_s);
           }
           imwrite(fixedFileName, timelapser->getDst());
       }
       else
       {
           blender->feed(img_warped_s, mask_warped, corners[img_idx]);
       }
   }

   if (!timelapse)
   {
       Mat result, result_mask;
       blender->blend(result, result_mask);

       // LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

       imwrite(result_name, result);
   }

   // LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
   return 0;
}

int main(int argc, char** argv) {
  getCalibrationDetails(numImage);
  homography();
  vector<string> images;

  for (int i = numImage; i > 0; i--) {
    string filename = "test" + to_string(i) + ".png";
    // Mat im = imread(filename);
    //
    // if (im.empty()) {
    //   cout << "Image " + filename + "is not found!" << endl;
    //   return ERROR;
    // }
    images.push_back(filename);
  }
  stitch(images);
  return 0;
  //
  // img_names.push_back("images/frame2.png");
  // img_names.push_back("images/frame1.png");
  // return findCameraPose();
}
