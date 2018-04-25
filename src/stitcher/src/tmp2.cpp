// Standard C++ I/O library.
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>


// OpenCV library.
#include <cv.h>
#include <highgui.h>

// OpenCV feature library.
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;

// Mat getCameraInfo(Mat info) {
//   ros::NodeHandle nh; // to get the private params
//   std::string camera_name = "head_camera";
//   std::string camera_info_url = "file:///home/donamphuong/.ros/camera_info/head_camera.yaml";
//   CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
//   // Get the saved camera info if any
//   sensor_msgs::CameraInfo cam_info_msg;
//
//   cam_info_msg = cam_info_manager.getCameraInfo();
// }

void basicPanoramaStitching(const string &img1Path, const string &img2Path)
{
    Mat img1 = imread(img1Path);
    Mat img2 = imread(img2Path);

    //! [camera-pose-from-Blender-at-location-1]
    Mat c1Mo = (Mat_<double>(4,4) << 0.9659258723258972, 0.2588190734386444, 0.0, 1.5529145002365112,
                                     0.08852133899927139, -0.3303661346435547, -0.9396926164627075, -0.10281121730804443,
                                     -0.24321036040782928, 0.9076734185218811, -0.342020183801651, 6.130080699920654,
                                     0, 0, 0, 1);
    //! [camera-pose-from-Blender-at-location-1]

    //! [camera-pose-from-Blender-at-location-2]
    Mat c2Mo = (Mat_<double>(4,4) << 0.9659258723258972, -0.2588190734386444, 0.0, -1.5529145002365112,
                                     -0.08852133899927139, -0.3303661346435547, -0.9396926164627075, -0.10281121730804443,
                                     0.24321036040782928, 0.9076734185218811, -0.342020183801651, 6.130080699920654,
                                     0, 0, 0, 1);
    //! [camera-pose-from-Blender-at-location-2]

    //! [camera-intrinsics-from-Blender]
    Mat cameraMatrix = (Mat_<double>(3,3) << 839.4042719889258, 0, 329.9572713427157,
                                              0, 833.8296950945119, 204.9791424679845,
                                              0, 0, 1);
    //! [camera-intrinsics-from-Blender]

    //! [extract-rotation]
    Mat R1 = c1Mo(Range(0,3), Range(0,3));
    Mat R2 = c2Mo(Range(0,3), Range(0,3));
    //! [extract-rotation]

    //! [compute-rotation-displacement]
    //c1Mo * oMc2
    Mat R_2to1 = R1*R2.t();
    //! [compute-rotation-displacement]

    //! [compute-homography]
    Mat H = cameraMatrix * R_2to1 * cameraMatrix.inv();
    H /= H.at<double>(2,2);
    cout << "H:\n" << H << endl;
    //! [compute-homography]

    //! [stitch]
    Mat img_stitch;
    warpPerspective(img2, img_stitch, H, Size(img2.cols*2, img2.rows));
    imshow("warpPerspective", img_stitch);
    Mat half = img_stitch(Rect(0, 0, img1.cols, img1.rows));
    img1.copyTo(half);
    //! [stitch]

    Mat img_compare;
    Mat img_space = Mat::zeros(Size(50, img1.rows), CV_8UC3);
    hconcat(img1, img_space, img_compare);
    hconcat(img_compare, img2, img_compare);
    imshow("Compare images", img_compare);

    imshow("Panorama stitching", img_stitch);
    // waitKey();
}


// main().
int main(int argv, char ** argc)
{
    cv::Mat im_ref, im_cmp;

    std::string  str_ref, str_cmp;

    // Read reference image.
    //std::cout<<"Input reference image filename: ";
    //std::cin>>str_ref;
    std::cout<<"-> Reading images."<<std::endl;
    str_ref = "images/ImageV0.jpg";

    im_ref = cv::imread(str_ref);
    // cv::imshow("Reference image", im_ref);

    // Read testing image.
    //std::cout<<"Input testing image filename: ";
    //std::cin>>str_cmp;
    str_cmp = "images/ImageV1.jpg";

    im_cmp = cv::imread(str_cmp);
    // cv::imshow("Testing image", im_cmp);

    // std::cout<<"Press any key to continue."<<std::endl;
    // cvWaitKey(0);

    // basicPanoramaStitching(str_ref, str_cmp);

  //  Feature detection.
    std::cout<<"-> Feature detection."<<std::endl;
    std::vector <cv::KeyPoint> key_ref, key_cmp;           // Vectors for features extracted from reference and testing images.
    cv::Mat  des_ref, des_cmp;                             // Descriptors for features of 2 images.

    Ptr<cv::Feature2D> sift = SIFT::create(400);                                          // An ORB object.

    sift->detectAndCompute(im_ref, cv::Mat(), key_ref, des_ref);             // Feature extraction.
    sift->detectAndCompute(im_cmp, cv::Mat(), key_cmp, des_cmp);


    // Show keypoints.
    std::cout<<"-> Show keypoints."<<std::endl;
    cv::Mat drawkey_ref, drawkey_cmp;                              // Output image for keypoint drawing.
    cv::drawKeypoints(im_ref, key_ref, drawkey_ref);               // Generate image for keypoint drawing.
    // cv::imshow("Keypoints of reference", drawkey_ref);
    cv::drawKeypoints(im_cmp, key_cmp, drawkey_cmp);
    // cv::imshow("Keypoints of test", drawkey_cmp);

    // cvWaitKey(0);


    // Matching.
    std::cout<<"-> Matching."<<std::endl;
    Ptr<DescriptorMatcher> matcher1 = DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> matches1;
    matcher1->match(des_ref, des_cmp, matches1);            // Match two sets of features.

    double max_dist = 0;
    double min_dist = 100;

    // Find out the minimum and maximum of all distance.
    for( int i = 0; i < des_ref.rows; i++ )
    {
        double dist = matches1[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    // cvWaitKey(0);


    // Eliminate relatively bad points.
    std::cout<<"-> Bad points elimination"<<std::endl;
    std::vector<cv::KeyPoint> kgood_ref, kgood_cmp;
    std::vector<cv::DMatch> goodMatch;
    for (int i=0; i<matches1.size(); i++)
    {
        if(matches1[i].distance < 2*min_dist)      // Keep points that are less than 2 times of the minimum distance.
        {
            goodMatch.push_back(matches1[i]);
            kgood_ref.push_back(key_ref[i]);
            kgood_cmp.push_back(key_cmp[i]);
        }  // end if
    } // end for
    // cvWaitKey(0);

    // Calculate affine transform matrix.
    std::cout<<"-> Calculating affine transformation. "<< goodMatch.size() << std::endl;
    std::vector<cv::Point2f>   frm1_feature, frm2_feature;
    const int p_size = goodMatch.size();
    // * tmpP = new tmpPoint[p_size];
    cv::Point2f tmpP;


    for(int i=0; i<goodMatch.size(); i++)
    {
        tmpP.x = kgood_ref[i].pt.x;
        tmpP.y = kgood_ref[i].pt.y;
        frm1_feature.push_back(tmpP);

        tmpP.x = kgood_cmp[i].pt.x;
        tmpP.y = kgood_cmp[i].pt.y;
        frm2_feature.push_back(tmpP);
    }
    //Draw top matches
    Mat imMatches;
    drawMatches(im_ref, key_ref, im_cmp, key_cmp, goodMatch, imMatches);
    std::cout << "Num matches " << goodMatch.size() << std::endl;
    imshow("img/matches.jpg", imMatches);
    // cv::Mat affine_mat = cv::estimateRigidTransform(frm1_feature, frm2_feature, true);
    cv::Mat im_transformed_cmp, im_transformed_ref;
    //
    // std::cout << "Size of affine matrix " << affine_mat.cols << ", " << affine_mat.rows << std::endl;
    // // Output results.
    // cv::warpAffine(im_cmp, im_transformed, affine_mat, cv::Size()); // error comes from here.
    // cv::imshow("Transformed image", im_transformed);
    // Find homography
     Mat h = findHomography( Mat(frm1_feature), Mat(frm2_feature), RANSAC);

  //    // Use homography to warp image
  //    warpPerspective(im_cmp, im_transformed_cmp, h, im_cmp.size());
  //    imshow("warped", im_transformed_cmp);
  // //   warpPerspective(im_ref, im_transformed_ref, h, im_ref.size());
  //
    std::vector<Mat> images;
    images.push_back(im_cmp);
    images.push_back(im_ref);
    Mat pano;
    Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA, true);
    Stitcher::Status status = stitcher->stitch(images, pano);

    if (status != Stitcher::OK) {
      std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
      return 1;
    }

    cv::imshow("Transformed image", pano);

    cvWaitKey(0);

    return 0;
}
