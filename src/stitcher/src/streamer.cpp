#include "headers/streamer.hpp"

#define HAS_WEBCAM

void CameraStreamer::captureFrame(int index) {
  VideoCapture *capture = camera_capture[index];
  // while (true) {
    clock_t start = clock();
    Mat frame, corrected;
    capture->read(frame);

    //undistort frame
    remap(frame, corrected, undistortMap1[index], undistortMap2[index], INTER_LINEAR, BORDER_CONSTANT);

    Mat img_warped, img_warped_s;
    // Warping image based on precomputed spherical map
    Rect image_roi = composedImageROI[index];
    img_warped.create(image_roi.height + 1, image_roi.width + 1,  corrected.type());
    remap(corrected, img_warped, composedImageUXMap[index], composedImageUYMap[index], INTER_LINEAR, BORDER_REFLECT);

    img_warped.convertTo(img_warped_s, CV_16S);

    float sharpness = 0.02f;

    feather_blend(index, img_warped_s, maps[index]);
    // place_images(index, img_warped_s);

    // frame_queue[index]->push(corrected);
    frame.release();
    double duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    // cout << "Capturing video frame from camera " << index << " takes: " << duration << endl;
  // }
}


void getUndistortMap() {
  Mat I = Mat_<double>::eye(3,3);

  for (int cam = 0; cam < numImage; cam++) {
    CalibrationDetails cal = calibrations[cam];
    initUndistortRectifyMap(cal.camera_matrix, cal.distortion, I, calibrations[0].camera_matrix, image_size, CV_16SC2, undistortMap1[cam], undistortMap2[cam]);
  }
}

void CameraStreamer::startMultiCapture() {
  isReady = false;
  VideoCapture *capture;
  thread *t;
  queue<Mat> *q;

  getUndistortMap();

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
      std::cout << "Camera " << std::to_string(i) << " cannot be opened!" << std::endl;
      exit(ERROR);
    }

    // t = new thread(&CameraStreamer::captureFrame, this, i);
    // camera_thread.push_back(t);

    // q = new queue<Mat>;
    // frame_queue.push_back(q);

    maps[i] = weight_maps[i].getMat(ACCESS_READ);
  }
  // cout << "hello" << endl;
}
