#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

using namespace cv;

image_transport::Publisher pub;

void save_frame(VideoCapture cap) {
  Mat frame;
  sensor_msgs::ImagePtr msg;
  cap >> frame;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    //cv::imwrite("images/ImageV", frame);
    pub.publish(msg);
    cv::waitKey(1);
  }
}

/*
This is a camera node that takes care of the communication with the camera
*/
int main(int argc, char** argv) {
  if (argc <= 1) return 1;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  pub = it.advertise("camera/image", 1);

  ros::Rate loop_rate(5);
  std::cout << "Please press ENTER to take a picture";

  int num_cam = atoi(argv[1]);
  cv::VideoCapture cap[num_cam + 1];

  for (int video_source = 1; video_source < num_cam + 1; video_source++) {
    cap[video_source-1].open(video_source);

    //Check if video device can be opened with the given index
    std::cout << cap[video_source-1].isOpened() << "\n";
    if (!cap[video_source-1].isOpened()) return 1;
  }

  while (nh.ok()) {
    if (std::cin.get() == '\n') {
      for (int i = 0; i < num_cam; i++) {
        save_frame(cap[i]);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}
