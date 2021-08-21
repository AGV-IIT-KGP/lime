cmake_minimum_required(VERSION 2.8.3)
project(transform_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(OpenCV 3)


# FOR C++11 SUPPORT
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(filter_points src/filter_points.cpp)
target_link_libraries(filter_points
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(encoder src/encoder.cpp)
target_link_libraries(encoder
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(correction_maintainer src/correction_maintainer.cpp)
target_link_libraries(correction_maintainer
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
)
add_executable(translate_points src/translate_points.cpp)
target_link_libraries(translate_points
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(error_saver src/error_saver.cpp)
target_link_libraries(error_saver
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
)