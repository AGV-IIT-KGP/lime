# LIME : Localisation in Mapped Environment
***LIME*** is an open source 3D LiDAR-based localisation framework. This framework enables localisation in a premapped environment. Prior mapping can be achieved by using packages like [loam_velodyne](https://github.com/laboshinl/loam_velodyne) and [LeGO_LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM). Wheel odometry is used to provide high frequency, low accuracy updates which are corrected by low frequency, high accuracy ICP corrections.

![example_image](https://i.ytimg.com/vi/flL5gf0JXh4/maxresdefault.jpg)

[video](https://www.youtube.com/watch?v=flL5gf0JXh4)

This package is built on top of the ROS ecosystem.

This package has been tested on Ubuntu 16.04 & ROS kinetic.
## Installation
***premapped_localization*** depends on the following libraries:
  - [PCL](https://github.com/PointCloudLibrary/pcl) : [Installation instructions](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Compilation.md)
  - [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) : [Installation instructions](http://pointclouds.org/downloads/linux.html)

Installing this package this package:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ShreyanshDarshan/premapped_localization.git

cd ~/catkin_ws/
catkin_make
```

## Examples
### 1. Using robot_localisation from this package with encoder data.
In a terminal execute:
```bash
cd ~/catkin_ws/
source devel/setup.bash
roslaunch icp setup.launch
```
If using a bagfile, in a different terminal, run:
```bash
rosbag play <path to bagfile>/<bagfile_name>.bag
```
Make sure that the encoder data is being published at the topic **/encoders** of message type geometry_msgs/Twist and is of the following format:
  - twist.linear.x -> left wheel encoder data
  - twist.linear.y -> right wheel encoder data
