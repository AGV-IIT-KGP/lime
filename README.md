# LIME : Localisation in Mapped Environment
***LIME*** is an open source 3D LiDAR-based localisation framework. This framework enables localisation in a premapped environment. Prior mapping can be achieved by using packages like [loam_velodyne](https://github.com/laboshinl/loam_velodyne) and [LeGO_LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM). Wheel odometry is used to provide high frequency, low accuracy updates which are corrected by low frequency, high accuracy ICP corrections.

![example_image](https://github.com/AGV-IIT-KGP/lime/blob/clean/lime_demo.gif)

[video](https://www.youtube.com/watch?v=flL5gf0JXh4)

This package is built on top of the ROS ecosystem.

This package has been tested on 
- Ubuntu 16.04, ROS kinetic
- Ubuntu 18.04, ROS melodic

## Installation
***lime*** depends on the following libraries:
  - [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) : [Installation instructions](https://libpointmatcher.readthedocs.io/en/stable/Compilation/)

Additionally, you might need to install these packages:
```bash
sudo apt-get install ros-melodic-tf2-sensor-msgs
sudo apt-get install ros-melodic-geographic-msgs
sudo apt-get install ros-melodic-move-base
```

To set up your catkin workspace, run:
```bash
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
catkin_make
```

To build ***lime***, run:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ShreyanshDarshan/premapped_localization.git

cd ~/catkin_ws/
catkin_make
```

## Example
### Using robot_localisation from this package with encoder data.
In a terminal execute:
```bash
cd ~/catkin_ws/
source devel/setup.bash
roslaunch transform_fusion setup.launch
```
If using a bagfile, in a different terminal, run:
```bash
rosbag play <path to bagfile>/<bagfile_name>.bag
```
Make sure that the encoder data is being published at the topic **/encoders** of message type geometry_msgs/Twist and is of the following format:
  - twist.linear.x -> left wheel encoder data
  - twist.linear.y -> right wheel encoder data
