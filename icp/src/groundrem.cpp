#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//#include <eigen3/Eigen>
#include <pcl/filters/extract_indices.h>
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
    tf::StampedTransform transform2;
    ros::init(argc, argv, "groundless_pt_gen");

    ros::NodeHandle node;
  
    ros::Rate rate(1000.0);
  
    tf::TransformListener listener;
    ros::Subscriber sub = node.subscribe("/velodyne_points", 1000, get_cloud);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/shifted_points", 1000);

    float zAvg = -1.4;
    // cout<<"Enter zavg"<<endl;
    // cin>>zAvg;

    while (node.ok())
    {
      pcl::PCLPointCloud2 groundless;
      pcl_conversions::toPCL(pcin,groundless);

      pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(groundless, *p_obstacles);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      
      for (int i = 0; i < (*p_obstacles).size(); i++)
      {
        pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y, p_obstacles->points[i].z);
        if (zAvg - pt.z > 0) // e.g. remove all pts below zAvg
        {
          inliers->indices.push_back(i);
        }
      }
      if ((*p_obstacles).size()>0)
      {
        extract.setInputCloud(p_obstacles);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*p_obstacles);
        pcl::toROSMsg(*p_obstacles.get(), pcin);

      }

      pcin.header.frame_id = "odom";
      
      cout<<transform1.getOrigin().x()<<endl;
      cloud_pub.publish(pcin);
      rate.sleep();
      ros::spinOnce();
    }
    return 0;
}