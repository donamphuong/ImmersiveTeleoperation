#include "streamer.hpp"

#define HAS_WEBCAM

void CameraStreamer::captureFrame(int index) {
  VideoCapture *capture = camera_capture[index];
  while (true) {
    clock_t start = clock();
    Mat frame, corrected;
    capture->read(frame);

    //undistort frame
    remap(frame, corrected, undistortMap1[index], undistortMap2[index], INTER_LINEAR, BORDER_CONSTANT);
    frame_queue[index]->push(corrected);

    frame.release();
    // double duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    // cout << "Capturing video frame from camera " << index << " takes: " << duration << endl;
  }
}

void CameraStreamer::startMultiCapture() {
  isReady = false;
  VideoCapture *capture;
  thread *t;
  queue<Mat> *q;

  for (int i = 0; i < numImage; i++) {
    int cam = i;
    #ifdef HAS_WEBCAM
      cam++;
    #endif
    capture = new VideoCapture(cam);
    capture->set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    capture->set(CAP_PROP_FRAME_WIDTH, 1920);
    capture->set(CAP_PROP_FRAME_HEIGHT, 1080);
    capture->set(CAP_PROP_AUTOFOCUS, true);

    camera_capture.push_back(capture);

    //Check if video device can be opened with the given index
    if (!capture->isOpened()) {
      std::cout << "Camera " << std::to_string(cam) << " cannot be opened!" << std::endl;
      exit(ERROR);
    }

    t = new thread(&CameraStreamer::captureFrame, this, i);
    camera_thread.push_back(t);

    q = new queue<Mat>;
    frame_queue.push_back(q);

    maps[i] = weight_maps[i].getMat(ACCESS_READ);
  }
  // cout << "hello" << endl;
}

CameraStreamer::~CameraStreamer() {
  stopMultiCapture();
}

void CameraStreamer::stopMultiCapture() {
  VideoCapture *cap;
  for (int i = 0; i < numImage; i++) {
    cap = camera_capture[i];
    if (cap->isOpened()) {
      //Release VideoCapture resource
      cap->release();
    }
  }
}
