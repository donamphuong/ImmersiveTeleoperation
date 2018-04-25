#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>

using namespace cv;

std::string to_string(int x) {
  std::stringstream stream;
  stream << x;
  return stream.str();
}

void save_frame(VideoCapture cap, std::vector<Mat> images, int view) {
  Mat frame;
  sensor_msgs::ImagePtr msg;
  cap >> frame;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    images.push_back(frame);
    imshow("view" + to_string(view), frame);
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
  int num_cam = atoi(argv[1]);

  cv::namedWindow("view1");
  cv::startWindowThread();
  cv::namedWindow("view2");
  cv::startWindowThread();
  ros::Rate loop_rate(5);
  std::cout << "Image Publisher running";

  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  image_transport::Publisher pub = it.advertise("camera/stitched_image", 1);
  cv::VideoCapture cap[num_cam];

  for (int video_source = 1; video_source < num_cam + 1; video_source++) {
    cap[video_source-1].open(video_source);

    //Check if video device can be opened with the given index
    std::cout << cap[video_source-1].isOpened() << "\n";
    if (!cap[video_source-1].isOpened()) return 1;
  }

  std::vector<Mat> images;
  while (nh.ok()) {
    for (int i = 0; i < num_cam; i++) {
      save_frame(cap[i], images, i+1);
    }

    // Mat pano;
    // //Converting ROS image message to an OpenCV image with BGR pixel encoding, then show it in a display window
    // Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA, true);
    // Stitcher::Status status = stitcher->stitch(images, pano);
    //
    // if (status != Stitcher::OK) {
    //   std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
    //   waitKey(1);
    //   // return 1;
    // }
    //
    // std::cout << "Images are Stitched";
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pano).toImageMsg();
    // pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
