#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>
#include "../../headers/stitch.hpp"
#include "streamer.hpp"
#include <tbb/tbb.h>
#include <opencv2/opencv.hpp>

#define NO_PARALLEL

using namespace cv;
image_transport::Publisher pub;

int test();
void getUndistortMap();
int run();
void save_frame(VideoCapture, std::vector<Mat>&, int);

int test() {
  clock_t start;
  double duration;
  vector<Mat> images;

  for (int i = 1; i < numImage + 1; i++) {
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
  Mat stitched;
  stitch(images, stitched);

  duration = (clock() - start) / (double) CLOCKS_PER_SEC;
  cout << "printf: " << duration << "\n";

  namedWindow("stitched", WINDOW_NORMAL);
  resizeWindow("stitched", 1024, 600);
  imshow ("stitched", stitched);
  waitKey();

  return 0;
}

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
  std::cout << "Image Publisher running" << std::endl;

  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  pub = it.advertise("camera", 1);

  namedWindow("stitched", WINDOW_NORMAL);
  resizeWindow("stitched", 1920, 1080);
  precomp();
  calibrations.clear();

  CameraStreamer multiCameras;
  multiCameras.startMultiCapture();

  while (nh.ok()) {
    clock_t start = clock();
    vector<Mat> images(numImage);
    map<int, Mat> imagesMap;

    double saveDuration = 0;
    numImage = 0;
    for (int i = 0; i < numImage; i++) {
      if (!multiCameras.frame_queue[i]->empty()) {
        images[i] = multiCameras.frame_queue[i]->front();
        multiCameras.frame_queue[i]->pop();
        numImage++;
      }
    }
    cout << "Total reading and undistorting image " << saveDuration << endl;

    clock_t startStitch = clock();
    Mat stitched;
    stitch(images, stitched);
    double duration = (clock() - startStitch) / (double) CLOCKS_PER_SEC;
    cout << "Stitching Time: " << duration << endl;

    imshow ("stitched", stitched);
    waitKey(1);
    cout << "Process time before publishing " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", stitched).toImageMsg();
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
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
