#include <iostream>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace pcl;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "correction_of_transform");
	ros::NodeHandle n;
	tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    
	
	
	while (ros::ok())
    {
        tf::StampedTransform odometry_stamped_transform;
        try{
                listener.lookupTransform("/base_link", "/odom",  ros::Time(0), odometry_stamped_transform);
            }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        br.sendTransform(tf::StampedTransform(corr_transform, ros::Time::now(), "odom", "cumm"));
	    ros::spinOnce();
    }
    return(0);
}