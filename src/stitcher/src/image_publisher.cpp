#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/stitching.hpp>
#include <string>

using namespace cv;

class CalibrationDetails {
  public:
  Mat camera_matrix;
  Mat distortion;
  Mat rectification;
  Mat projection;
};

std::vector<CalibrationDetails> calibrations;

std::string to_string(int x) {
  std::stringstream stream;
  stream << x;
  return stream.str();
}

void getCalibrationDetails(int num_cam) {
  for (int i = 1; i < num_cam+1; i++) {
    CalibrationDetails cal;
    std::string filename = "/home/donamphuong/ImmersiveTeleoperation/src/stitcher/calibration/camera" + to_string(i) + ".yaml";
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
      std::cout << "Could not open the configuration file" << std::endl;
      exit(-1) ;
    }

    fs["camera_matrix"] >> cal.camera_matrix;
    fs["distortion_coefficients"] >> cal.distortion;
    fs["rectification_matrix"] >> cal.rectification;
    fs["projection_matrix"] >> cal.projection;
    fs.release();

    calibrations.push_back(cal);
  }
}

void save_frame(VideoCapture cap, std::vector<Mat>& images, int cam) {
  Mat frame;
  sensor_msgs::ImagePtr msg;
  cap >> frame;
  //Check if the grabbed frame is actually full with some content
  if (!frame.empty()) {
    Mat corrected;

    undistort(frame, corrected, calibrations[cam].camera_matrix, calibrations[cam].distortion);
    images.push_back(corrected);
    imshow("view" + to_string(cam+1), corrected);
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
  getCalibrationDetails(num_cam);

  // cv::namedWindow("view1");
  // cv::startWindowThread();
  // cv::namedWindow("view2");
  // cv::startWindowThread();
  ros::Rate loop_rate(20);
  std::cout << "Image Publisher running" << std::endl;

  //this let master tell any nodes listening on 'camera/image' that we are going to publish data on that topic.
  //This will buffer up to 1 message before beginning to throw away old ones
  image_transport::Publisher pub = it.advertise("camera", 1);
  // cv::VideoCapture cap[num_cam];
  //
  // for (int video_source = 1; video_source < num_cam + 1; video_source++) {
  //   cap[video_source-1].open(video_source);
  //
  //   //Check if video device can be opened with the given index
  //   if (!cap[video_source-1].isOpened()) return 1;
  // }

  Mat pano = imread("result.jpg");
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pano).toImageMsg();

  while (nh.ok()) {
    // std::vector<Mat> images;
    // for (int i = 0; i < num_cam; i++) {
    //   save_frame(cap[i], images, i);
    // }

    // Mat pano;
    // //Converting ROS image message to an OpenCV image with BGR pixel encoding, then show it in a display window
    // Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA, true);
    // Stitcher::Status status = stitcher->stitch(images, pano);
    //
    // if (status != Stitcher::OK) {
    //   std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
    //   return 1;
    // }

    // imshow("stitched images", pano);

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
