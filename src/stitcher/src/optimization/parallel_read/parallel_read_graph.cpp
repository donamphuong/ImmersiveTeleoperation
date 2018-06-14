#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>
#include "headers/stitch.hpp"
#include "headers/streamer.hpp"
#include <tbb/tbb.h>
#include <opencv2/opencv.hpp>
#include <thread>

#define NO_PARALLEL

using namespace cv;

image_transport::Publisher pub;

int test();
void getUndistortMap();
int run();
void save_frame(VideoCapture, std::vector<Mat>&, int);

void getUndistortMap() {
  Mat I = Mat_<double>::eye(3,3);

  for (int cam = 0; cam < numImage; cam++) {
    CalibrationDetails cal = calibrations[cam];
    initUndistortRectifyMap(cal.camera_matrix, cal.distortion, I, calibrations[0].camera_matrix, image_size, CV_16SC2, undistortMap1[cam], undistortMap2[cam]);
  }
}

int run() {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ros::Rate loop_rate(20);
  ROS_INFO("Image Publisher running");

  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  pub = it.advertise("camera", 1);
  CameraStreamer multiCameras;
  multiCameras.startMultiCapture();

  precomp();
  calibrations.clear();
  string results = "";

  for (int iter = 0; iter < 100; iter++) {
  //while (nh.ok()) {
    clock_t start = clock();
    // vector<Mat> images;
    map<int, Mat> imagesMap;

    double saveDuration = 0;
    for (int i = 0; i < numImage; i++) {
      thread *t = new thread(&CameraStreamer::captureFrame, multiCameras, i);
      // release(t);
      // save_frame(cap[i], images, i);
      t->join();
    }
    //cout << "Total reading and undistorting image " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;

    clock_t startStitch = clock();
    Mat stitched;
    stitch(images, stitched);
    double duration = (clock() - startStitch) / (double) CLOCKS_PER_SEC;
    //cout << "Stitching Time: " << duration << endl;

    imshow ("stitched", stitched);
    waitKey(1);
    results += to_string((clock() - start) / (double) CLOCKS_PER_SEC) + "\t";

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", stitched).toImageMsg();
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  results += "\n";

  ofstream myfile;
  myfile.open("parallel_read.txt");
  myfile << results;
  myfile.close();
}

//Show contents of cameras after calibrations
void save_frame(VideoCapture cap, std::vector<Mat >&images, int cam) {
  Mat frame;
  clock_t start;
  double duration;

  start = clock();
  cap >> frame;
  cout << "Saving time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    Mat corrected /*= Mat(image_size.width, image_size.height, CV_8UC3)*/;

    start = clock();
    remap(frame, corrected, undistortMap1[cam], undistortMap2[cam], INTER_LINEAR, BORDER_CONSTANT);

    #ifdef DEBUG
      duration = (-start + clock()) / (double) CLOCKS_PER_SEC;
      cout << "Undistorting: " << duration << endl;
    #endif

    images[cam] = (corrected);
    frame.release();
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
  getUndistortMap();

  // return test();
  return run();
}
