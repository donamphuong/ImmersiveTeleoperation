#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>
#include "stitch.cpp"
// #include "details.cpp"

using namespace cv;

image_transport::Publisher pub;

int test();
int run();
void save_frame(VideoCapture, std::vector<Mat>&, int);

int test() {
  clock_t start;
  double duration;
  vector<Mat> images;

  for (int i = numImage; i > 0; i--) {
    string filename = "test" + to_string(i) + ".png";
    Mat im = imread(filename);
    if (im.empty()) {
      cout << "File " << filename << " does not exist" << endl;
      return ERROR;
    }
    images.push_back(im);
  }

  precomp();
  start = clock();
  Mat stitched = stitch(images);
  duration = (clock() - start) / (double) CLOCKS_PER_SEC;
  cout << "printf: " << duration << "\n";

  namedWindow("stitched", WINDOW_NORMAL);
  resizeWindow("stitched", 1024, 600);
  imshow ("stitched", stitched);
  waitKey();

  return 0;
}

int run() {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ros::Rate loop_rate(20);
  std::cout << "Image Publisher running" << std::endl;

  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  pub = it.advertise("camera", 1);
  cv::VideoCapture cap[numImage];

  for (int video_source = 1; video_source < numImage + 1; video_source++) {
    string gst = "v4l2src device=/dev/video" + std::to_string(video_source) + " ! video/x-raw, format=BGR, width=1920, height=1080 ! appsink";
    cap[video_source-1].open(gst);

    //Check if video device can be opened with the given index
    if (!cap[video_source-1].isOpened()) {
      std::cout << "Device " << std::to_string(video_source-1) << " cannot be opened!" << std::endl;
      return ERROR;
    }
  }

  namedWindow("stitched", WINDOW_NORMAL);
  resizeWindow("stitched", 1024, 600);
  precomp();

  while (nh.ok()) {
    std::vector<Mat> images;
    for (int i = 0; i < numImage; i++) {
      save_frame(cap[i], images, i);
    }

    clock_t start = clock();
    Mat stitched = stitch(images);
    double duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    cout << "Stitching Time: " << duration << endl;

    images.clear();
    imshow ("stitched", stitched);
    waitKey(1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", stitched).toImageMsg();
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

//Show contents of cameras after calibrations
void save_frame(VideoCapture cap, std::vector<Mat>& images, int cam) {
  Mat frame;
  cap >> frame;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    Mat corrected;

    undistort(frame, corrected, calibrations[cam].camera_matrix, calibrations[cam].distortion);
    images.push_back(corrected);
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", corrected).toImageMsg();
    // pub.publish(msg);
  }
}

/*
This is a camera node that takes care of the communication with the camera
*/
int main(int argc, char** argv) {
  // if (argc <= 1) return 1;
  ros::init(argc, argv, "image_publisher");
  // numImage = atoi(argv[1]);

  getCalibrationDetails();

  // return test();
  return run();
}
