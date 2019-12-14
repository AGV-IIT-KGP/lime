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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#define ITERATION_COUNT 5000

using namespace std;
using namespace pcl;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

const float leaf = 0.4f;
const float fac = 2.0f;
const float NUM_SAMPS = 50;
const float K = 5;

ofstream file_1("transforms.txt");
// if (pcl::io::loadPCDFile<pcl::PointXYZ> ("map.pcd", *cloud_out) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read the map file i.e map.pcd \n");
//     return (-1);
//   }
int i=0;


PointCloudT::Ptr cloud_out (new PointCloudT);
pcl::PCLPointCloud2 pcl_pc2;
PointCloudT::Ptr transformed_cloud (new PointCloudT);
FeatureCloudT::Ptr cloud_out_features (new FeatureCloudT);

void do_icp()
{
  PointCloudT::Ptr current_lidar (new PointCloudT);
  FeatureCloudT::Ptr current_lidar_features (new FeatureCloudT);
  pcl::fromPCLPointCloud2(pcl_pc2, *current_lidar);

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (current_lidar);
  grid.filter (*current_lidar);

  // Estimate normals for cloud_out
  pcl::console::print_highlight ("Estimating cur_lid normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  // nest.setRadiusSearch (leaf*fac);
  nest.setKSearch(K);
  nest.setInputCloud (current_lidar);
  nest.compute (*current_lidar);

  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  // fest.setRadiusSearch (leaf*fac);
  fest.setKSearch(K);
  fest.setInputCloud (current_lidar);
  fest.setInputNormals (current_lidar);
  fest.compute (*current_lidar_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (current_lidar);
  align.setSourceFeatures (current_lidar_features);
  align.setInputTarget (cloud_out);
  align.setTargetFeatures (cloud_out_features);
  align.setMaximumIterations (ITERATION_COUNT); // Number of RANSAC iterations
  align.setNumberOfSamples (NUM_SAMPS); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  pcl::console::print_highlight ("Alignment started\n");
  {
    pcl::ScopeTime t("Alignment");
    align.align (*transformed_cloud);
  }
  
  cout<<"Descriptor size: "<<current_lidar_features->size()<<endl;
  cout<<"Number of points: "<<current_lidar->size()<<endl;
  int sz = current_lidar->size();
  for (int i=0; i<current_lidar->size(); i++)
  {
    cout<<"Points"<<(*current_lidar)[i].normal_x<<std::endl;
  }
}

void pc2_to_pcl_plus_icp(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
  pcl_conversions::toPCL(*input,pcl_pc2);
}
int main (int argc, char** argv)
{
	ros::init(argc, argv, "RANSAC_on_map");
  pcl::io::loadPCDFile<PointNT> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/map.pcd", *cloud_out);

  pcl::VoxelGrid<PointNT> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud_out);
  grid.filter (*cloud_out);

  pcl::console::print_highlight ("Estimating cloud_out normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  // nest.setRadiusSearch (leaf*fac);
  nest.setKSearch(5);
  nest.setInputCloud (cloud_out);
  nest.compute (*cloud_out);
  
  FeatureEstimationT fest;
  // fest.setRadiusSearch (leaf*fac);
  fest.setKSearch(5);
  fest.setInputCloud (cloud_out);
  fest.setInputNormals (cloud_out);
  fest.compute (*cloud_out_features);

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/shifted_points", 1000, pc2_to_pcl_plus_icp);
    ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);
    while (ros::ok())
    {
        sensor_msgs::PointCloud2 current_lidar_msg;
        do_icp();
        pcl::toROSMsg(*transformed_cloud.get(),current_lidar_msg );
        current_lidar_msg.header.frame_id = "odom";
        chatter_pub.publish(current_lidar_msg);
        //cout<<current_lidar_msg;
	    ros::spinOnce();
    }
	return 0;
}