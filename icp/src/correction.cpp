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
#include <pcl/conversioint main (int argc, char** argv)ns.h>
#include <pcl_ros/transforms.h>
using namespace std;
using namespace pcl;
tf::Transform transform2;
nav_msgs::Odometry uncorrected; 
void initialize_correct_odometry(const nav_msgs::Odometry input)
{
    uncorrected=input;
}
void tf_to_nav_msgs(const tf:Transform input_tf)
{
    transform2=input_tf;
    tf::Matrix3x3 m(transform2.getRotation());
    double r,p,y;
    m.getRPY(r,p,y);
    nav_msgs::Odometry corrected;
    corrected.pose.pose.position.x=transform2.getOrigin().x();
    corrected.pose.pose.position.y=transform2.getOrigin().y();
    corrected.pose.pose.position.z=transform2.getOrigin().z();
    corrected.pose.pose.orientation.x=
    corrected.pose.pose.orientation.y=
    corrected.pose.pose.orientation.z=
    corrected.pose.pose.orientation.w=

}
int main (int argc, char** argv)
{
    ros::init(argc, argv, "correction_of_transform");
    //pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/map.pcd", *cloud_out);
	ros::NodeHandle n;
	ros::Subscriber odometry_sub = n.subscribe("/odometry/filtered_odom", 1000, initialize_correct_odometry  );
    ros::Subscriber tf_sub = n.subscribe("/corr_matrix", 1000,tf_to_nav_msgs );
    ros::Publisher chatter_pub = n.advertise<nav_msgs/Odometry>("/odometry/filtered_new", 1000);

    while (ros::ok())
    {
        nav_msgs::Odometry object_msg;
        corrected=uncorrected;
        //find_matrix();
        //pcl::toROSMsg(*transformed_cloud.get(),object_msg );
        object_msg.header.frame_id = "odom";
        chatter_pub.publish(object_msg);
        //cout<<object_msg;
	    ros::spinOnce();
    }
    retunr(0);
}