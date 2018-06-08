#include "details.hpp"
#include <thread>
#include <queue>
#include <vector>
#include <opencv2/opencv.hpp>
#include "stitch.hpp"

class CameraStreamer {
  public:
    vector<VideoCapture*> camera_capture;
    vector<queue<Mat>*> frame_queue;
    vector<thread*> camera_thread;
    bool isReady;
    // ~CameraStreamer();

  public:
    void startMultiCapture();
    // void stopMultiCapture();
    void captureFrame(int index);
};

vector<Mat> undistortMap1(numImage);
vector<Mat> undistortMap2(numImage);
vector<Mat> maps(numImage);

