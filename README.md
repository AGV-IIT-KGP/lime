# premapped_localization
***premapped_localization*** is an open source 3D LiDAR-based localisation framework. This framework enables localisation in a premapped environment. Prior mapping can be achieved by using packages like [loam_velodyne](https://github.com/laboshinl/loam_velodyne) and [LeGO_LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM). Wheel odometry is used to provide high frequency, low accuracy updates which are corrected by low frequency, high accuracy ICP corrections.

[video](https://www.youtube.com/watch?v=flL5gf0JXh4)

This package is built on top of the ROS ecosystem.

This package has been tested on Ubuntu 16.04 & ROS kinetic.
## Installation
***premapped_localization*** depends on the following libraries:
  - [PCL](https://github.com/PointCloudLibrary/pcl) : [Installation instructions](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Compilation.md)
  - [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) : [Installation instructions](http://pointclouds.org/downloads/linux.html)

HOW TO USE:
Clone this repo in the src folder of catkin_ws and then run catkin_make
