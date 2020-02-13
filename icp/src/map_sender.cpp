#include <iostream>
#include "ros/ros.h"
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
#include <pcl_ros/transforms.h>

using namespace std;

ofstream file_1("transforms.txt");

int i=0;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cld(new pcl::PointCloud<pcl::PointXYZ>);

int main (int argc, char** argv)
{
	ros::init(argc, argv, "map_publisher");
    // ros::package::getPath('PKG_NAME')
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/high res map/kitti final map.pcd", *map_cld);
	ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1000);
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cld.get(), map_msg );
    map_msg.header.frame_id = "odom";
    //while (ros::ok())
    {
        map_pub.publish(map_msg);
        //cout<<object_msg;
	    ros::spinOnce();
    }
	return 0;
}