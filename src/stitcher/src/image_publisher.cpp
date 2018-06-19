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

vector<UMat> undistortMap1(numImage);
vector<UMat> undistortMap2(numImage);

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
    initUndistortRectifyMap(cal.camera_matrix, cal.distortion, I, Mat(), image_size, undistortMap1[cam].type(), undistortMap1[cam], undistortMap2[cam]);
  }
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
    clock_t start = clock();
    clock_t startStitch = clock();
    vector<Mat> images(numImage);
    map<int, Mat>  imagesMap;

    parallel_for_(Range(0, numImage), SaveFrame(cap, imagesMap));

    for (map<int, Mat>::iterator i = imagesMap.begin(); i != imagesMap.end(); i++) {
      images[i->first] = i->second;
    }
    cout << "Undistorting: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;

    startStitch = clock();
    Mat stitched;
    stitch(images, stitched);
    double duration = (clock() - startStitch) / (double) CLOCKS_PER_SEC;
    cout << "Stitching Time: " << duration << endl;

    images.clear();
    imshow ("stitched", stitched);
    waitKey(1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", stitched).toImageMsg();
    pub.publish(msg);

    cout << "Process time before publishing " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//Show contents of cameras after calibrations
void save_frame(VideoCapture cap, std::vector<Mat>& images, int cam) {
  Mat frame;
  clock_t start = clock();
  cap >> frame;
  cout << "saving time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    Mat corrected;

    #ifdef DEBUG
      clock_t start = clock();
    #endif

    remap(frame, corrected, undistortMap1[cam], undistortMap2[cam], INTER_LINEAR, BORDER_CONSTANT);

    #ifdef DEBUG
      cout << "Undistorting: " << (start - clock()) / (double) CLOCKS_PER_SEC << endl;
    #endif

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
  getUndistortMap();

  // return test();
  return run();
}
