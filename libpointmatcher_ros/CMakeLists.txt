cmake_minimum_required(VERSION 2.8.3)

project(libpointmatcher_ros)

find_package(catkin REQUIRED COMPONENTS
pcl_conversions
pcl_ros
roscpp
sensor_msgs
nav_msgs
tf
tf_conversions
eigen_conversions
)

# This was tested for a system install of libpointmatcher
find_package(libpointmatcher CONFIG)


catkin_package(
INCLUDE_DIRS include
LIBRARIES pointmatcher_ros
CATKIN_DEPENDS
roscpp
sensor_msgs
nav_msgs
tf
tf_conversions
eigen_conversions
libpointmatcher
)


# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
# set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
    add_definitions(-std=gnu++0x)
  else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
    message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
  endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
#   message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)

# well, this breaks libpointmatcher on non-unix platform,
# please contribute the define for your favorite compiler
# It must support (compiler support in parenthesis):
# - N1984: auto-typed variables (gcc, clang)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DHAVE_YAML_CPP)
add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${libpointmatcher_INCLUDE_DIRS}
  ${POINTMATCHER_INCLUDE_DIRS}
)

add_library(pointmatcher_ros src/point_cloud.cpp src/transform.cpp)

#target_link_libraries(pointmatcher_ros ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})
add_executable(pmicp src/pcl_ros_icp.cpp)
target_link_libraries(pmicp pointmatcher_ros ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES} ${roslib_LIBRARIES})
#target_link_libraries(pmicp pointmatcher)
install(TARGETS pmicp RUNTIME DESTINATION bin)

#target_link_libraries(pointmatcher_ros ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})
add_executable(ICP_new src/icp_lpm.cpp)
target_link_libraries(ICP_new pointmatcher_ros ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES} ${roslib_LIBRARIES})
#target_link_libraries(pmicp pointmatcher)
install(TARGETS ICP_new RUNTIME DESTINATION bin)