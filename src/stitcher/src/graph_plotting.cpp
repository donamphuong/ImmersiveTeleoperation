#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>
#include "headers/stitch.hpp"
// #include "headers/streamer.hpp"
#include <opencv2/opencv.hpp>
#include <thread>

#define NO_PARALLEL

using namespace cv;

image_transport::Publisher pub;
vector<Mat> undistortMap1(numImage);
vector<Mat> undistortMap2(numImage);

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

  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  cv::VideoCapture cap[numImage];

  for (int video_source = 0; video_source < numImage; video_source++) {
    cap[video_source].open(video_source);
    cap[video_source].set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    cap[video_source].set(CAP_PROP_FRAME_WIDTH, 1920);
    cap[video_source].set(CAP_PROP_FRAME_HEIGHT, 1080);
    cap[video_source].set(CAP_PROP_AUTOFOCUS, true);

    //Check if video device can be opened with the given index
    if (!cap[video_source].isOpened()) {
      std::cout << "Device " << std::to_string(video_source) << " cannot be opened!" << std::endl;
      return ERROR;
    }
  }

  precomp();
  calibrations.clear();
  string results = "";
  string stitchTime = "";
  string stream = "";

  for (int iter = 0; iter < 100; iter++) {
  //while (nh.ok()) {
    clock_t start = clock();
    vector<Mat> images(numImage);
    map<int, Mat> imagesMap;

    double streamDuration = 0;
    for (int i = 0; i < numImage; i++) {
      save_frame(cap[i], images, i);
    }
    streamDuration = (clock() - start) / (double) CLOCKS_PER_SEC;

    clock_t startStitch = clock();
    Mat stitched;
    stitch(images, stitched);
    double duration = (clock() - startStitch) / (double) CLOCKS_PER_SEC;
    // cout << "Stitching Time: " << duration << endl;

    stitchTime += to_string(duration) + "\t";
    stream += to_string(streamDuration) + "\t";
    results += to_string((clock() - start) / (double) CLOCKS_PER_SEC) + "\t";
  }
  results += "\n";

  ofstream resultTotal;
  resultTotal.open("resultTotal.txt");
  resultTotal << results;
  resultTotal.close();

  ofstream resultStitch;
  resultStitch.open("resultStitch.txt");
  resultStitch << stitchTime;
  resultStitch.close();

  ofstream resultStream;
  resultStream.open("resultStream.txt");
  resultStream << stream;
  resultStream.close();
}
