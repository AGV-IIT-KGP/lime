#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

int main (int argc, char** argv)
{
	ros::init(argc, argv, "listener");
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/map_lane_manual_new.pcd", *cloud_out);
	ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/unshifted_cloud", 1000);
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*cloud_out.get(), map_msg );
    while (ros::ok())
    {
        map_msg.header.frame_id = "/odom";
        map_msg.header.stamp=ros::Time::now();
        map_pub.publish(map_msg);
        //cout<<object_msg;
	    ros::spinOnce();
    }
	return 0;
}