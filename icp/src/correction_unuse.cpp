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
//tf::Transform transform2;
nav_msgs::Odometry corrected; 
void initialize_correct_odometry(const nav_msgs::Odometry input)
{
    corrected=input;
}
// void tf_to_nav_msgs(const tf:Transform input_tf)
// {
//     transform2=input_tf;
//     tf::Matrix3x3 m(transform2.getRotation());
//     double r,p,y;
//     m.getRPY(r,p,y);
//     nav_msgs::Odometry corrected;
//     corrected.pose.pose.position.x=transform2.getOrigin().x();
//     corrected.pose.pose.position.y=transform2.getOrigin().y();
//     corrected.pose.pose.position.z=transform2.getOrigin().z();
//     corrected.pose.pose.orientation.x=transform1.getRotation().x();
//     corrected.pose.pose.orientation.y=
//     corrected.pose.pose.orientation.z=
//     corrected.pose.pose.orientation.w=

// }
int main (int argc, char** argv)
{
    ros::init(argc, argv, "correction_of_transform");
    //pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/map.pcd", *cloud_out);
	ros::NodeHandle n;
	ros::Subscriber odometry_sub = n.subscribe("/odometry/filtered_odom", 1000, initialize_correct_odometry);
    //ros::Subscriber tf_sub = n.subscribe("/corr_matrix", 1000,tf_to_nav_msgs );
    ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("/odometry/filtered_new", 1000);
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    while (ros::ok())
    {
        //nav_msgs::Odometry object_msg;
        tf::StampedTransform cumm_transform;
        try{
                listener.lookupTransform("cumm", "/base_link",  ros::Time(0), cumm_transform);
            }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        cout<<"aaaaaaaaaaaa"<<endl;
        tf::StampedTransform odometry_stamped_transform;
        try{
                listener.lookupTransform("/base_link", "/odom",  ros::Time(0), odometry_stamped_transform);
            }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // ros::Time odom_transform_time = odometry_stamped_transform.stamp_;

         //odometry_stamped_transform = tf::StampedTransform(corr_transform*odometry_stamped_transform, odometry_stamped_transform.stamp_, "odom", "corr");	
        tf::Transform corr_transform=corr_transform*odometry_stamped_transform;

        br.sendTransform(tf::StampedTransform(corr_transform, ros::Time::now(), "odom", "cumm"));
        corrected.pose.pose.position.x=odometry_stamped_transform.getOrigin().x();
        corrected.pose.pose.position.y=odometry_stamped_transform.getOrigin().y();
        corrected.pose.pose.position.z=odometry_stamped_transform.getOrigin().z();
        corrected.pose.pose.orientation.x=odometry_stamped_transform.getRotation().x();
        corrected.pose.pose.orientation.y=odometry_stamped_transform.getRotation().y();
        corrected.pose.pose.orientation.z=odometry_stamped_transform.getRotation().z();
        corrected.pose.pose.orientation.w=odometry_stamped_transform.getRotation().w();

        //find_matrix();
        //pcl::toROSMsg(*transformed_cloud.get(),object_msg );
        corrected.header.frame_id = "odom";
        chatter_pub.publish(corrected);
        //cout<<object_msg;
	    ros::spinOnce();
    }
    return(0);
}