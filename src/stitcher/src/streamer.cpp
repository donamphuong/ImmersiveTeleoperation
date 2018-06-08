#include "headers/streamer.hpp"

void CameraStreamer::captureFrame(int index) {
  VideoCapture *capture = camera_capture[index];
  // while (true) {
    Mat frame, corrected;
    capture->read(frame);

    remap(frame, corrected, undistortMap1[index], undistortMap2[index], INTER_LINEAR, BORDER_CONSTANT);
    // frame_queue[index]->push(corrected);
    images[index] = (corrected);
    frame.release();
  // }
}

void CameraStreamer::startMultiCapture() {
  VideoCapture *capture;
  thread *t;
  queue<Mat> *q;
  for (int i = 0; i < numImage; i++) {
    capture = new VideoCapture(i+1);
    capture->set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    capture->set(CAP_PROP_FRAME_WIDTH, 1920);
    capture->set(CAP_PROP_FRAME_HEIGHT, 1080);
    capture->set(CAP_PROP_AUTOFOCUS, true);

    camera_capture.push_back(capture);

    //Check if video device can be opened with the given index
    if (!capture->isOpened()) {
      std::cout << "Camera " << std::to_string(i) << " cannot be opened!" << std::endl;
      exit(ERROR);
    }

    // t = new thread(&CameraStreamer::captureFrame, this, i);
    // camera_thread.push_back(t);

    // q = new queue<Mat>;
    // frame_queue.push_back(q);
  }
  // cout << "hello" << endl;
}
