#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace camera_info_manager;

int MAX_FEATURES = 500;
float GOOD_MATCH_PERCENT = 0.15;

void alignImages(Mat &img1, Mat &img2, Mat &imgResult, Mat &h) {
  //Convert images to grayscale
  Mat img1Gray, img2Gray;
  cvtColor(img1, img1Gray, CV_BGR2GRAY);
  cvtColor(img2, img2Gray, CV_BGR2GRAY);

  //Variables to store keypoints and descriptors
  std::vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;

  //Detect SIFT features and compute descriptors
  Ptr<Feature2D> surf = SURF::create(MAX_FEATURES);
  surf->detectAndCompute(img1Gray, Mat(), keypoints1, descriptors1);
  surf->detectAndCompute(img2Gray, Mat(), keypoints2, descriptors2);

  // //Match features
  std::vector<DMatch> matches;
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
  matcher->match(descriptors1, descriptors2, matches, Mat());

  //Sort matches by score
  std::sort(matches.begin(), matches.end());

  //Remove not so good matches
  const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
  matches.erase(matches.begin() + numGoodMatches, matches.end());

  //Draw top matches
  Mat imMatches;
  drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
  imwrite("img/matches.jpg", imMatches);

  //Extract location of good matches
  std::vector<Point2f> points1, points2;

  for (size_t i = 0; i < matches.size(); i++) {
    points1.push_back(keypoints1[matches[i].queryIdx].pt);
    points2.push_back(keypoints2[matches[i].queryIdx].pt);
  }

  //Find homography
  h = findHomography(points1, points2, RANSAC);

  Mat imgMask = Mat(img1.size(), CV_8UC1, Scalar(255));
  Mat imgMaskWarped, imgTrainWarped;
  //Use homography to warp image
  warpPerspective(imgMask, imgMaskWarped, h, img2.size());
  warpPerspective(img1, imgTrainWarped, h, img2.size());

  imgTrainWarped.copyTo(img2, imgMaskWarped);
  imwrite("Later.jpg", imgTrainWarped);
}

float angleBetween(Point &v1, Point &v2) {
  float len1 = sqrt(v1.x * v1. x + v1.y * v1.y);
  float len2 = sqrt(v2.x * v2. x + v2.y * v2.y);

  float dot = v1.x * v2.x + v1.y * v2.y;
  float a = dot/(len1 * len2);

  if (a >= 1.0) {
    return 0.0;
  } else if (a <= -1.0) {
    return 3.14;
  } else {
    return acos(a);
  }
}

cv::Point2f convert_pt(cv::Point2f point,int w,int h) {
//center the point at 0,0
cv::Point2f pc(point.x-w/2,point.y-h/2);

//these are your free parameters
float f = w;
float r = w;

float omega = w/2;
float z0 = f - sqrt(r*r-omega*omega);

float zc = (2*z0+sqrt(4*z0*z0-4*(pc.x*pc.x/(f*f)+1)*(z0*z0-r*r)))/(2* (pc.x*pc.x/(f*f)+1));
cv::Point2f final_point(pc.x*zc/f,pc.y*zc/f);
final_point.x += w/2;
final_point.y += h/2;
return final_point;
}

void cylindrical_project(Mat &image, float width, float height, Mat &dest_im) {
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
        cv::Point2f current_pos(x,y);
        current_pos = convert_pt(current_pos, width, height);

        cv::Point2i top_left((int)current_pos.x,(int)current_pos.y); //top left because of integer rounding

        //make sure the point is actually inside the original image
        if(top_left.x < 0 ||
           top_left.x > width-2 ||
           top_left.y < 0 ||
           top_left.y > height-2) {
            continue;
        }

        //bilinear interpolation
        float dx = current_pos.x-top_left.x;
        float dy = current_pos.y-top_left.y;

        float weight_tl = (1.0 - dx) * (1.0 - dy);
        float weight_tr = (dx)       * (1.0 - dy);
        float weight_bl = (1.0 - dx) * (dy);
        float weight_br = (dx)       * (dy);

        uchar value =   weight_tl * image.at<uchar>(top_left) +
        weight_tr * image.at<uchar>(top_left.y,top_left.x+1) +
        weight_bl * image.at<uchar>(top_left.y+1,top_left.x) +
        weight_br * image.at<uchar>(top_left.y+1,top_left.x+1);

        dest_im.at<uchar>(y,x) = value;
    }
  }
}

int main(int argc, char **argv) {
  std::vector<Mat> imgs;
  Mat img1 = imread("images/Image0.jpg");
  Mat img2 = imread("images/2.jpg");
  ros::init(argc, argv, "image_alignment");
 ros::NodeHandle nh; // to get the private params
 ros::NodeHandle _nh("~"); // to get the private params

 std::string camera_name = "head_camera";
 std::string camera_info_url = "file:///home/donamphuong/.ros/camera_info/head_camera.yaml";
  // _nh.param("camera_info_url", camera_info_url, std::string(""));

  CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
  // Get the saved camera info if any
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg = cam_info_manager.getCameraInfo();

  detail::CylindricalWarper warper(0.5);
  Mat h, dest;
  float K[3][3], R[3][3];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3;j ++) {
      K[i][j] = cam_info_msg.K[j + i*3];
      R[i][j] = cam_info_msg.R[j + i*3];
    }
  }

  warper.warp(img1, Mat(3, 3, CV_32F, K), Mat(3, 3, CV_32F, R), 0, BORDER_DEFAULT, dest);
  // alignImages(img1, img2, dest, h);
  imwrite("Later.jpg", dest);
  return 0;
}
