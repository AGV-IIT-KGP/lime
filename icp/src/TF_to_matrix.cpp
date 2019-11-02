#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <bits/stdc++.h>

tf::StampedTransform transform;

// pcl::PointCloud<pcl::PointXYZ>::Ptr pcin (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr pcout (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud pcin;
sensor_msgs::PointCloud pcout;

void placeholder(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
  sensor_msgs::PointCloud2 temp = *input;
  sensor_msgs::convertPointCloud2ToPointCloud (temp, pcin);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_matrix");

  ros::NodeHandle node;

  tf::TransformListener listener;

  // ros::Rate rate(10.0);

  ros::Subscriber sub = node.subscribe("/velodyne_points", 1000, placeholder);
  ros::Publisher chatter_pub = node.advertise<sensor_msgs::PointCloud>("/shifted_cloud", 1000);
  
  while (node.ok()){
    
    // try{
    //   listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    //sensor_msgs::PointCloud2 object_msg;    
    listener.transformPointCloud ("base_link", pcin, pcout);
    chatter_pub.publish(pcout);
    //std::cout<<transform.getOrigin().x()<<"\t"<<transform.getOrigin().y()<<std::endl;
    //turtlesim::Velocity vel_msg;
    // vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
    //                             transform.getOrigin().x());
    // vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
    //                             pow(transform.getOrigin().y(), 2));
    //turtle_vel.publish(vel_msg);
    //chatter_pub.publish(object_msg);
    //rate.sleep();
    ros::spinOnce();
  }
  return 0;
};