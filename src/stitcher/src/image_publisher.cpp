#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>
#include "stitch.cpp"

using namespace cv;

vector<Mat> undistortMap1(numImage);
vector<Mat> undistortMap2(numImage);

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
    initUndistortRectifyMap(cal.camera_matrix, cal.distortion, I, Mat(), image_size, CV_16SC2, undistortMap1[cam], undistortMap2[cam]);
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
  cv::VideoCapture cap[numImage];

  for (int video_source = 1; video_source < numImage + 1; video_source++) {
    cap[video_source-1].open(video_source);
    cap[video_source-1].set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    cap[video_source-1].set(CAP_PROP_FRAME_WIDTH,1920);
    cap[video_source-1].set(CAP_PROP_FRAME_HEIGHT,1080);
    cap[video_source-1].set(CAP_PROP_AUTOFOCUS,true);


    //Check if video device can be opened with the given index
    if (!cap[video_source-1].isOpened()) {
      std::cout << "Device " << std::to_string(video_source-1) << " cannot be opened!" << std::endl;
      return ERROR;
    }
  }

  namedWindow("stitched", WINDOW_NORMAL);
  resizeWindow("stitched", 1920, 1080);
  precomp();
  calibrations.clear();


  while (nh.ok()) {
    clock_t start = clock();
    clock_t startStitch = clock();
    std::vector<Mat> images;
    for (int i = 0; i < numImage; i++) {
      save_frame(cap[i], images, i);
    }

    startStitch = clock();
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

//Show contents of cameras after calibrations
void save_frame(VideoCapture cap, std::vector<Mat >&images, int cam) {
  Mat frame;
  clock_t start;
  double duration;
  cap >> frame;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    Mat corrected = Mat(image_size.width, image_size.height, CV_8UC3);

    start = clock();
    remap(frame, corrected, undistortMap1[cam], undistortMap2[cam], INTER_LINEAR, BORDER_CONSTANT);

    // #ifdef DEBUG
      duration = (-start + clock()) / (double) CLOCKS_PER_SEC;
      cout << "Undistorting: " << duration << endl;
    // #endif

    images.push_back(corrected);
    frame.release();
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
  getUndistortMap();

  // return test();
  return run();
}
