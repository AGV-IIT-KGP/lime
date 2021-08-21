// Author : Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include <bits/stdc++.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

using namespace std;

// Pointcloud for subscribing to input cloud
sensor_msgs::PointCloud2 pcin;

// Subscriber function to accept input cloud
void getCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input) {
  pcin = *input;
  return;
}

// Removes ground points from input cloud
sensor_msgs::PointCloud2 removeGround(sensor_msgs::PointCloud2 input_cloud,
                                      int z_thresh) {
  sensor_msgs::PointCloud2 output_cloud;
  pcl::PCLPointCloud2 groundless;
  pcl_conversions::toPCL(input_cloud, groundless);
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundless_P(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(groundless, *groundless_P);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Iterate over all points inn the input cloud
  for (int i = 0; i < (*groundless_P).size(); i++) {
    pcl::PointXYZ pt(groundless_P->points[i].x, groundless_P->points[i].y,
                     groundless_P->points[i].z);
    // Remove all pts below z_thresh
    if (z_thresh - pt.z > 0) {
      inliers->indices.push_back(i);
    }
  }

  // use the indices in the inliers array to filter the cloud
  if ((*groundless_P).size() > 0) {
    extract.setInputCloud(groundless_P);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*groundless_P);
    pcl::toROSMsg(*groundless_P.get(), output_cloud);
  }

  // set header of the groundless cloud and return
  output_cloud.header.frame_id = "odom";
  return output_cloud;
}

// Applies voxel filter to the input cloud and returns the filtered cloud
sensor_msgs::PointCloud2 voxelFilter(sensor_msgs::PointCloud2 input_cloud,
                                     float leafxy, float leafz) {
  sensor_msgs::PointCloud2 output_cloud;
  // Convert input from sensor_msgs::PointCloud2 to fromPCLPointCloud2
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_P(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input_cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *output_cloud_P);

  // Set leaf size and apply voxel filter
  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(leafxy, leafxy, leafz);
  grid.setInputCloud(output_cloud_P);
  grid.filter(*output_cloud_P);

  // Return voxel filtered output cloud
  pcl::toROSMsg(*output_cloud_P.get(), output_cloud);
  return output_cloud;
}

// Returns distance between point a and b
float dist(pcl::PointXYZ a, pcl::PointXYZ b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
         (a.z - b.z) * (a.z - b.z);
}

// Extracts and returns edge points from the input cloud
sensor_msgs::PointCloud2 extractEdges(sensor_msgs::PointCloud2 input_cloud,
                                      int K, float thresholdDistance) {
  sensor_msgs::PointCloud2 output_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cld(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 pcl_pc2;

  // Convert input from sensor_msgs::PointCloud2 to fromPCLPointCloud2
  pcl_conversions::toPCL(input_cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *unfiltered);
  (*edge_cld).clear();
  pcl::fromPCLPointCloud2(pcl_pc2, *unfiltered);

  // Set up a KD tree on the input cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(unfiltered);
  pcl::PointXYZ search_point;

  cout << endl << "edge extraction started" << endl;

  // K nearest neighbor search
  std::vector<int> point_idx_NKN_search(K);
  std::vector<float> point_NKN_squared_distance(K);

  std::cout << "K nearest neighbor search at (" << search_point.x << " "
            << search_point.y << " " << search_point.z << ") with K=" << K
            << std::endl;

  int sz = (*unfiltered).size();
  int edge_pts = 0, non_edge_pts = 0;

  // Iterate over all points
  for (int iter = 0; iter < (*unfiltered).size(); iter++) {
    search_point = unfiltered->points[iter];
    // Use KD tree to search for K nearest neighbours to a point
    if (kdtree.nearestKSearch(search_point, K, point_idx_NKN_search,
                              point_NKN_squared_distance) > 0) {
      pcl::PointXYZ centroid;
      centroid.x = centroid.y = centroid.z = 0;
      int num = 0;
      float res_dist = 10000;
      for (std::size_t i = 0; i < point_idx_NKN_search.size(); ++i) {
        if (dist(search_point, unfiltered->points[point_idx_NKN_search[i]]) >
            0.001) {
          centroid.x += unfiltered->points[point_idx_NKN_search[i]].x;
          centroid.y += unfiltered->points[point_idx_NKN_search[i]].y;
          centroid.z += unfiltered->points[point_idx_NKN_search[i]].z;
          num++;
          float res_check =
              dist(search_point, unfiltered->points[point_idx_NKN_search[i]]);
          if (res_check < res_dist) {
            res_dist = res_check;
          }
        }
      }
      if (num > 0) {
        centroid.x /= num;
        centroid.y /= num;
        centroid.z /= num;
      }

      // If point is too far from neighbourhood centroid, it is edge point
      if (dist(centroid, search_point) > thresholdDistance * res_dist) {
        (*edge_cld).push_back(search_point);
        edge_pts++;
      } else {
        non_edge_pts++;
      }
    }
  }
  cout << endl
       << "ended\nedge pts = " << edge_pts
       << "\nnon edge pts = " << non_edge_pts << endl;

  // Set header of output cloud and return
  pcl::toROSMsg(*edge_cld.get(), output_cloud);
  output_cloud.header.frame_id = "odom";
  return output_cloud;
}

int main(int argc, char **argv) {

  // Initialise ros node
  ros::init(argc, argv, "filter_points");

  ros::NodeHandle node;

  ros::Rate rate(1000.0);

  // Subscribe to the translated points and publish to /filtered_points
  ros::Subscriber sub = node.subscribe("/shifted_points", 1000, getCloud);
  ros::Publisher cloud_pub =
      node.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1000);

  // Read constants from arguments and convert to variables
  float z_thresh = stof(argv[1]);
  float leaf_size = stof(argv[2]);
  int K = stoi(argv[3]);
  int edge_thresh_dist = stoi(argv[4]);

  while (node.ok()) {
    // Apply voxel filter on input cloud
    sensor_msgs::PointCloud2 voxelised_cloud =
        voxelFilter(pcin, leaf_size, leaf_size);
    // Apply edge filter on voxelised cloud
    sensor_msgs::PointCloud2 edge_cloud =
        extractEdges(voxelised_cloud, K, edge_thresh_dist);
    // Remove ground on the edge cloud
    sensor_msgs::PointCloud2 groundless_cloud =
        removeGround(edge_cloud, z_thresh);
    // Publish the final cloud
    cloud_pub.publish(groundless_cloud);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}