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
using namespace pcl;
ofstream file_1("transforms.txt");
// if (pcl::io::loadPCDFile<pcl::PointXYZ> ("map.pcd", *cloud_out) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read the map file i.e map.pcd \n");
//     return (-1);
//   }
int i=0;
Eigen::Matrix4f cumm_transform;
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2 pcl_pc2;
void do_pcl()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_lidar(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *current_lidar);
    
  	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  	icp.setInputSource(current_lidar);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    //icp.getFitnessScore() << std::endl;
    std::cout<<i++;
    // file_1<<i<<")"<< icp.getFinalTransformation() <<"\n\n";



   // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // Eigen::Affine3f transform_2=icp.getFinalTransformation();
    Eigen::Matrix4f transform_1 = icp.getFinalTransformation();
    cumm_transform=transform1*cumm_transform;

    tf::Vector3 origin;
    origin.setValue(static_cast<double>(cumm_transform(0,3)),static_cast<double>(cumm_transform(1,3)),static_cast<double>(cumm_transform(2,3)));

    cout << origin << endl;
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(cumm_transform(0,0)), static_cast<double>(cumm_transform(0,1)), static_cast<double>(cumm_transform(0,2)), 
            static_cast<double>(cumm_transform(1,0)), static_cast<double>(cumm_transform(1,1)), static_cast<double>(cumm_transform(1,2)), 
            static_cast<double>(cumm_transform(2,0)), static_cast<double>(cumm_transform(2,1)), static_cast<double>(cumm_transform(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);
    // Define a translation of 2.5 meters on the x axis.
    //transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    //transform_2.rotate (Eigen::AngleAxisf (0.1, Eigen::Vector3f::UnitZ()));

    // // Print the transformation
    // printf ("\nMethod #2: using an Affine3f\n");
    // cout << transform_1.matrix() << std::endl;

    // // Executing the transformation
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*current_lidar, *transformed_cloud, transform);
}

void pc2_to_pcl_plus_icp(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
        pcl_conversions::toPCL(*input,pcl_pc2);
//     pcl::PCLPointCloud2 pcl_pc2;
//     pcl_conversions::toPCL(*input,pcl_pc2);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr current_lidar(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(pcl_pc2,*current_lidar);
    
//   	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   	icp.setInputSource(current_lidar);
//     icp.setInputTarget(cloud_out);
//     pcl::PointCloud<pcl::PointXYZ> Final;
//     icp.align(Final);
//     //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//     //icp.getFitnessScore() << std::endl;
//     std::cout<<i++;
//     // file_1<<i<<")"<< icp.getFinalTransformation() <<"\n\n";



//    // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//     // Eigen::Affine3f transform_2=icp.getFinalTransformation();
//     Eigen::Matrix4f transform_1 = icp.getFinalTransformation();
//     // Define a translation of 2.5 meters on the x axis.
//     //transform_2.translation() << 2.5, 0.0, 0.0;

//     // The same rotation matrix as before; theta radians around Z axis
//     //transform_2.rotate (Eigen::AngleAxisf (0.1, Eigen::Vector3f::UnitZ()));

//     // // Print the transformation
//     // printf ("\nMethod #2: using an Affine3f\n");
//     // cout << transform_1.matrix() << std::endl;

//     // // Executing the transformation
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//     // // You can either apply transform_1 or transform_2; they are the same
//     pcl::transformPointCloud (*current_lidar, *transformed_cloud, transform_1);

    
}
int main (int argc, char** argv)
{
	ros::init(argc, argv, "ICP_on_map");
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/map.pcd", *cloud_out);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/shifted_points", 1000, pc2_to_pcl_plus_icp);
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);
    ros::Publisher tf_pub = node.advertise<tf::Transform>("/corr_matrix", 1000);
    cumm_transform << 1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1;
    tf_pub.publish(cumm_transform);
    while (ros::ok())
    {
        sensor_msgs::PointCloud2 object_msg;
        do_pcl();
        pcl::toROSMsg(*transformed_cloud.get(),object_msg );
        object_msg.header.frame_id = "odom";
        cloud_pub.publish(object_msg);
        tf_pub.publish(cumm_transform);
        //cout<<object_msg;
	    ros::spinOnce();
    }
	return 0;
}