#include <iostream>
#include <cstdio>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <vector>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "../details.cpp"
#include <tbb/tbb.h>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

using namespace std;
using namespace cv;
using namespace tbb;


void turnOnVideos() {
  getCalibrationDetails();
  VideoCapture cap1;
  cap1.open(1);
  VideoCapture cap2;
  cap2.open(2);

  cap1.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  cap1.set(CAP_PROP_FRAME_WIDTH,1920);
  cap1.set(CAP_PROP_FRAME_HEIGHT,1080);
  cap1.set(CAP_PROP_AUTOFOCUS,true);

  cap2.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  cap2.set(CAP_PROP_FRAME_WIDTH,1920);
  cap2.set(CAP_PROP_FRAME_HEIGHT,1080);
  cap2.set(CAP_PROP_AUTOFOCUS,true);

  namedWindow("frame1", WINDOW_NORMAL);
  resizeWindow("frame1", 1024, 600);
  namedWindow("frame2", WINDOW_NORMAL);
  resizeWindow("frame2", 1024, 600);

  while (true) {
    Mat frame1;
    cap1 >> frame1;
    Mat frame2;
    cap2 >> frame2;

    Mat corrected1, corrected2;

    clock_t start = clock();
    undistort(frame1, corrected1, calibrations[0].camera_matrix, calibrations[0].distortion);
    cout << "undistort time: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;
    imshow("frame1", frame1);
    cv::waitKey(1);
    undistort(frame2, corrected2, calibrations[0].camera_matrix, calibrations[1].distortion);
    imshow("frame2", frame2);
    cv::waitKey(1);

  }
}

vector<VideoCapture*> camera_capture;
vector<queue<Mat>*> frame_queue;
vector<thread*> camera_thread;
void captureFrame(int idx) {
  VideoCapture *capture = camera_capture[idx];
  while (true) {
    Mat frame;
    (*capture) >> frame;
    frame_queue[idx]->push(frame);
    frame.release();
  }
  thread *t;
  queue<Mat> *q;
  for (int i = 0; i < numImage; i++)
  { 
    //Put VideoCapture to the vector
    camera_capture.push_back(new VideoCapture(i));
    
    //Make thread instance
    t = new thread(captureFrame, this, i);
    
    //Put thread to the vector
    camera_thread.push_back(t);
    
    //Make a queue instance
    q = new queue<Mat>;
    
    //Put queue to the vector
    frame_queue.push_back(q);
  }
}

void startMultipleCapture() {
  VideoCapture *capture;
  thread *t;
  queue<Mat> *q;
  for (int i = 0; i < numImage; i++)
  { 
    //Put VideoCapture to the vector
    camera_capture.push_back(new VideoCapture(i));
    
    //Make thread instance
    t = new thread(captureFrame, this, i);
    
    //Put thread to the vector
    camera_thread.push_back(t);
    
    //Make a queue instance
    q = new queue<Mat>;
    
    //Put queue to the vector
    frame_queue.push_back(q);
  }
}

int main(int argc, char** argv) {
  getCalibrationDetails();
startMultipleCapture();
  while(true) {
    //Retrieve frames from each camera capture thread
    for (int i = 0; i < capture_source.size(); i++)
    {
      Mat frame;
      //Pop frame from queue and check if the frame is valid
      if (cam.frame_queue[i]->try_pop(frame))
      {
          //Show frame on Highgui window
          imshow(label[i], frame);
      }
    }
    
  }

}
