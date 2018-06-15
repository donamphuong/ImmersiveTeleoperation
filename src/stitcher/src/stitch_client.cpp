#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include <queue>
#include <sstream>
#include "headers/details.hpp"

using namespace cv;
using namespace std;

image_transport::Publisher pub;
vector<queue<Mat>*> frames;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void callback1(const sensor_msgs::ImageConstPtr& msg) {
  frames[0]->push(cv_bridge::toCvShare(msg, "bgr8")->image);
}

void callback2(const sensor_msgs::ImageConstPtr& msg) {
  frames[1]->push(cv_bridge::toCvShare(msg, "bgr8")->image);
}

void callback3(const sensor_msgs::ImageConstPtr& msg) {
  frames[2]->push(cv_bridge::toCvShare(msg, "bgr8")->image);
}

void callback4(const sensor_msgs::ImageConstPtr& msg) {
  frames[3]->push(cv_bridge::toCvShare(msg, "bgr8")->image);
}

void callback5(const sensor_msgs::ImageConstPtr& msg) {
  frames[4]->push(cv_bridge::toCvShare(msg, "bgr8")->image);
}

void callback6(const sensor_msgs::ImageConstPtr& msg) {
  frames[5]->push(cv_bridge::toCvShare(msg, "bgr8")->image);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_processing");
  ros::NodeHandle nh;

  message_filters::Subscriber<Image> sub1(nh, "camera1", 1);
  message_filters::Subscriber<Image> sub2(nh, "camera2", 1);
  message_filters::Subscriber<Image> sub3(nh, "camera3", 1);
  message_filters::Subscriber<Image> sub4(nh, "camera4", 1);
  message_filters::Subscriber<Image> sub5(nh, "camera5", 1);
  message_filters::Subscriber<Image> sub6(nh, "camera6", 1);



  //Display our window
  ros::spin();
  cv::destroyWindow("view");
}