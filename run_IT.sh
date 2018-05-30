#!/bin/bash
catkin_make

valgrind --tool=callgrind rosrun stitcher image_publisher
