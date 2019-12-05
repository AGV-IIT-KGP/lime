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
    ros::init(argc, argv, "shift_for_ICP");

    ros::NodeHandle node;
  
    ros::Rate rate(1000.0);
  
    tf::TransformListener listener;
    ros::Subscriber sub = node.subscribe("/velodyne_points", 1000, get_cloud);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/shifted_points", 1000);
    while (node.ok())
    {
        
        try
        {
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform1);
        //cout<<transform.translation.x<<endl<<transform.translation.y<<endl<<endl;
        }
        catch (tf::TransformException ex)
        {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
        tf::Matrix3x3 m(transform1.getRotation());
        double r,p,y;
        m.getRPY(r,p,y);
        //y = -y;
        // r = -r;
        // p = -p;
        Eigen::Matrix4f Tm, Rm;
        Rm <<     1,   0,   0,  0,
                0,   1,    0,  0,
                0,   0,   1,    0,
                0,            0,          0,       1; 
        Tm <<     1,   0,   0,  transform1.getOrigin().x(),
                0,   1,    0,  transform1.getOrigin().y(),
                0,   0,   1,    transform1.getOrigin().z(),
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
        //std::cout<<Rm<<std::endl;
        pcl_ros::transformPointCloud (Rm, pcin, pcout);
        pcl_ros::transformPointCloud (Tm, pcout, pcout2);
        pcout2.header.frame_id = "odom";
        cloud_pub.publish(pcout2);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}