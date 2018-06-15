#ifndef STITCH_HPP_
#define STITCH_HPP_

#include "precomp.hpp"
#include <tbb/tbb.h>
#include <tbb/task_group.h>
#include <tbb/parallel_invoke.h>

vector<Mat> maps(numImage);

#define DEBUG
void place_images(int img_idx, Mat &img_warped_s) ;
void feather_blend(int img_idx, Mat &img_warped_s, Mat &weight_map) ;
void normalize_blended_image() ;
void stitch(vector<Mat> &full_images, Mat &result);
void clearCanvas();
void assembleCanvas();
#endif
