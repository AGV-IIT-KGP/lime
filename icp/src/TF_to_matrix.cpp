#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//#include <eigen3/Eigen>

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
sensor_msgs::PointCloud2 pcout2;
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
  
  ros::Rate rate(1000.0);
  
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
    std::cout<<transform1.getOrigin().x()<<" "<<transform1.getOrigin().y()<<" "<<transform1.getRotation()<<std::endl;
    tf::Matrix3x3 m(transform1.getRotation());
    double r,p,y;
    m.getRPY(r,p,y);
    y = -y;
    // r = -r;
    // p = -p;
    Eigen::Matrix4f Tm, Rm;
    Rm <<     1,   0,   0,  0,
              0,   1,    0,  0,
              0,   0,   1,    0,
              0,            0,          0,       1; 
    Tm <<     1,   0,   0,  -transform1.getOrigin().x(),
              0,   1,    0,  -transform1.getOrigin().y(),
              0,   0,   1,    -transform1.getOrigin().z(),
              0,            0,          0,       1; 
    Rm(0,0)=cos(y)*cos(p);
    Rm(0,1)=(cos(y)*sin(p)*sin(r))-(sin(y)*cos(r));
    Rm(0,2)=(cos(y)*sin(p)*cos(r))+(sin(y)*sin(r));
    Rm(1,0)=sin(y)*cos(p);
    Rm(1,1)=(sin(y)*sin(p)*sin(r))+(cos(y)*cos(r));
    Rm(1,2)=(sin(y)*sin(p)*cos(r))-(cos(y)*sin(r));
    Rm(2,0)=-sin(p);
    Rm(2,1)=cos(p)*sin(r);
    Rm(2,2)=cos(p)*cos(r);
        std::cout<<Rm<<std::endl;

    if (i>4)
    {
      std::cout<<1<<std::endl;
      //pcl_ros::transformPointCloud ("base_link", pcin, pcout, listener);

      pcl_ros::transformPointCloud (Tm, pcin, pcout);
      pcl_ros::transformPointCloud (Rm, pcout, pcout2);
    }
    pcout2.header.frame_id = "/base_link";
    cloud_pub.publish(pcout2);

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