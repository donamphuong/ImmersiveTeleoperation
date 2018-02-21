#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int count = 0;

/* 
The callback function that will get called when a new image has arrived on the "camera/image". Callback function only handles normal sensor_msgs/Image type.
*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    if (count == 0) {
    //Converting ROS image message to an OpenCV image with BGR pixel encoding, then show it in a display window
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cvWaitKey(30);
    count++;
  }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_processing");
  ros::NodeHandle nh;

  //Create an OpenCV display window
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  //Display our window
  ros::spin();
  cv::destroyWindow("view");
}
