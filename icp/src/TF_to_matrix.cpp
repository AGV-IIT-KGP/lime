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

using namespace std;

// pcl::PointCloud<pcl::PointXYZ>::Ptr pcin (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr pcout (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 pcin;
sensor_msgs::PointCloud2 pcout;

void get_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
  // sensor_msgs::PointCloud2 pcin = *input;
  // sensor_msgs::convertPointCloud2ToPointCloud (temp, pcin);
  pcin = *input;
  return;
}

int main(int argc, char** argv)
{
  tf::StampedTransform transform1;
  ros::init(argc, argv, "shift");

  ros::NodeHandle node;
  
  ros::Rate rate(2.0);
  
  tf::TransformListener listener;

  // ros::Rate rate(10.0);

  ros::Subscriber sub = node.subscribe("/unshifted_cloud", 1000, get_cloud);
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/cloud_in", 1000);
  int i=0;
  while (node.ok())
  {
    i++;
    try
    {
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform1);
      //cout<<transform.translation.x<<endl<<transform.translation.y<<endl<<endl;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    sensor_msgs::PointCloud2 buffer_local;
    std::cout<<transform1.getOrigin().x()<<" "<<transform1.getOrigin().y()<<std::endl;

    if (i>4)
    {
      std::cout<<1<<std::endl;
      //pcl_ros::transformPointCloud ("base_link", pcin, pcout, listener);
      pcl_ros::transformPointCloud ("base_link", pcin, pcout, listener);
    }
    pcout.header.frame_id = "/odom";
    cloud_pub.publish(pcout);

    //sensor_msgs::PointCloud2 object_msg;    
      //listener.transformPointCloud ("base_link", pcin, pcout);
      //chatter_pub.publish(pcout);
    //chatter_pub.publish(object_msg);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
};

/*
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  int i=0;
  while (node.ok()){
    i++;
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    std::cout<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<std::endl;

    if (i>20)
    {
      std::cout<<1<<std::endl;
      sensor_msgs::PointCloud2 buffer_local;
      pcl_ros::transformPointCloud ("base_link", pcin, pcout, listener);
    }
    // transformPointCloud("/base_laser",pcin, pcout, listener);
    //cloud_pub.publish(pcout);

    // turtlesim::Velocity vel_msg;
    // vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
    //                             transform.getOrigin().x());
    // vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
    //                             pow(transform.getOrigin().y(), 2));
    // turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};

*/