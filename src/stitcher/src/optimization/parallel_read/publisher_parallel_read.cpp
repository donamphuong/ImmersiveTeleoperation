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
#include <thread>

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

class SaveFrame : public cv::ParallelLoopBody {
  private:
    VideoCapture *cap;
    map<int, Mat> &images;

  public:
    SaveFrame(VideoCapture *inputCap, map<int, Mat> &inputImages)
        :cap(inputCap), images(inputImages) {}

    virtual void operator()(const cv::Range& range) const {
      for(int i = range.start; i < range.end; i++) {
        Mat frame;
        clock_t start;
        double duration;
        cap[i] >> frame;
        //Check if the grabbed frame is actually full with some content
        if (!frame.empty()) {
          Mat corrected = Mat(image_size.width, image_size.height, CV_8UC3);

          start = clock();
          remap(frame, corrected, undistortMap1[i], undistortMap2[i], INTER_LINEAR, BORDER_CONSTANT);

          // #ifdef DEBUG
            duration = (-start + clock()) / (double) CLOCKS_PER_SEC;
            cout << "Undistorting: " << duration << endl;
          // #endif

          images.insert(pair <int, Mat>(i, corrected));
          frame.release();
          // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", corrected).toImageMsg();
          // pub.publish(msg);
        }
      }
    }
};

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
  CameraStreamer multiCameras;
  multiCameras.startMultiCapture();

  // cv::VideoCapture cap[numImage];
  //
  //  for (int video_source = 0; video_source < numImage; video_source++) {
  //    cap[video_source].open(video_source);
  //    cap[video_source].set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  //    cap[video_source].set(CAP_PROP_FRAME_WIDTH, 1920);
  //    cap[video_source].set(CAP_PROP_FRAME_HEIGHT, 1080);
  //    cap[video_source].set(CAP_PROP_AUTOFOCUS, true);
  //
  //    //Check if video device can be opened with the given index
  //    if (!cap[video_source].isOpened()) {
  //      std::cout << "Device " << std::to_string(video_source) << " cannot be opened!" << std::endl;
  //      return ERROR;
  //    }
  //  }

  precomp();
  calibrations.clear();

  while (nh.ok()) {
    clock_t start = clock();
    // vector<Mat> images;
    map<int, Mat> imagesMap;

    double saveDuration = 0;
    vector<thread*> threads;
    for (int i = 0; i < numImage; i++) {
      thread *t = new thread(&CameraStreamer::captureFrame, multiCameras, i);
      // release(t);
      // save_frame(cap[i], images, i);
      threads.push_back(t);
      t->join();
    }
    cout << "Total reading and undistorting image " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;

    for (int i = 0; i < numImage; i++) {
      delete(threads[i]);
    }

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
