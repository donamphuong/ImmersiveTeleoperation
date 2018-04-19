#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>

using namespace cv;

int count = 0;
image_transport::Publisher pub;
/*
The callback function that will get called when a new image has arrived on the "camera/image". Callback function only handles normal sensor_msgs/Image type.
*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    return;
  }

  //currently only stitches two pictures together
  if (count == 0) {
    imwrite("images/Image1.jpg", cv_ptr->image);
    imshow("view", cv_ptr->image);
    waitKey(1);
  } else {
    std::vector<Mat> multipleImages;
    Mat stitchedImage;

    multipleImages.push_back(imread("images/Image1.jpg"));
    multipleImages.push_back(cv_ptr->image);

    //Converting ROS image message to an OpenCV image with BGR pixel encoding, then show it in a display window
    Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA, false);
    Stitcher::Status status = stitcher->stitch(multipleImages, cv_ptr->image);

    if (status != Stitcher::OK) {
      std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
      return;
    }

    imwrite("images/Panorama.jpg", cv_ptr->image);
    waitKey(3);

    pub.publish(cv_ptr->toImageMsg());
  }
  count++;
}

/*
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
     cv::waitKey(30);
  } catch (cv_bridge::Exception& e) {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "image_processing");
  ros::NodeHandle nh;

  //Create an OpenCV display window
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);


  pub = it.advertise("camera/stitched_image", 1);

  //Display our window
  ros::spin();
  cv::destroyWindow("view");
}
