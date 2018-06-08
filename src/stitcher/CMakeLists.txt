cmake_minimum_required(VERSION 2.8.3)
project(stitcher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET (CMAKE_C_FLAGS "-O3")
SET (CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #std_msgs
  image_transport
  cv_bridge
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(CUDA_FOUND)
     #collect CUDA files
     FILE(GLOB_RECURSE CUDA_SRC  *.cu)
     #build static library
     CUDA_ADD_LIBRARY(my_cuda_lib ${CUDA_SRC} STATIC)
     SET(LIBS ${LIBS} ${my_cuda_lib})
endif()

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stitcher
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## To allow newest version of OpenCV to be used
## OpenCV 4.0 supports GPU functions
#set(catkin_LIBRARIES #/opt/ros/kinetic/lib/libimage_transport.so;/opt/ros/kinetic/lib/libmessage_filters.so;/usr/lib/x86_64-linux-gnu/libtinyxml2.so;/opt/ros/kinetic/lib/libclass_loader.so;/usr/lib/libPocoFoundation.so;/usr/lib/x86_64-linux-gnu/libdl.so;/opt/ros/kinetic/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libboost_signals.so;/opt/ros/kinetic/lib/libxmlrpcpp.so;/opt/ros/kinetic/lib/libroslib.so;/opt/ros/kinetic/lib/librospack.so;/usr/lib/x86_64-linux-gnu/libpython2.7.so;/usr/lib/x86_64-linux-gnu/libboost_files#ystem.so;/usr/lib/x86_64-linux-gnu/libboost_program_options.so;/usr/lib/x86_64-linux-gnu/libtinyxml.so;/opt/ros/kinetic/lib/libcv_bridge.so;/opt/ros/kinetic/lib/librosconsole.so;/opt/ros/kinetic/lib/librosconsole_log4cxx.so;/opt/ros/kinetic/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/liblog4cxx.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/opt/ros/kinetic/lib/libroscpp_serialization.so;/opt/ros/kinetic/lib/librostime.so;/opt/ros/kinetic/lib/libcpp_common.so;/usr/lib/x86_6#4-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so
#)
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_30,code=sm_30;
    --std=c++11
    )

add_library(details src/details.cpp)
add_library(stitch src/stitch.cpp)
add_library(precomp src/precomp.cpp )
add_library(streamer src/streamer.cpp)
#cuda_add_library(test src/test/test.cu)

target_link_libraries(precomp
  ${OpenCV_LIBRARIES})

add_executable(image_publisher src/image_publisher.cpp)
target_link_libraries(image_publisher
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
  stitch
  details
  streamer
  precomp
)
add_dependencies(image_publisher stitcher_generate_messages_cpp)

add_executable(graph src/graph_plotting.cpp)
target_link_libraries(graph
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
  stitch
  details
  streamer
  precomp
)
add_dependencies(graph stitcher_generate_messages_cpp)


add_executable(image_processing src/test/image_processing.cpp)
target_link_libraries(image_processing
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(image_processing stitcher_generate_messages_cpp)

add_executable(image_testing src/test/image_testing.cpp)
target_link_libraries(image_testing
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  -ltbb
  details
  precomp
  stitch
)
add_dependencies(image_testing stitcher_generate_messages_cpp)
