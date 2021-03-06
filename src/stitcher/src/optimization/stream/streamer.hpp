#include "../../headers/details.hpp"
#include <thread>
#include <queue>
#include <vector>
#include <opencv2/opencv.hpp>
#include "../../headers/stitch.hpp"

class CameraStreamer {
  public:
    vector<VideoCapture*> camera_capture;
    vector<queue<Mat>*> frame_queue;
    vector<thread*> camera_thread;
    bool isReady;
    //Destructor for releasing resource(s)
    ~CameraStreamer();

  public:
    void startMultiCapture();
    void stopMultiCapture();
    void captureFrame(int index);
};

vector<Mat> undistortMap1(numImage);
vector<Mat> undistortMap2(numImage);
// vector<Mat> maps(numImage);
vector<Mat> images(numImage);
