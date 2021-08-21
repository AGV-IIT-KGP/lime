// Author: Radhika Patwawri, Shreyansh Darshan (
// https://github.com/ShreyanshDarshan )

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "ros/ros.h"
#include <bits/stdc++.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace pcl;

// To store the map pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr
    map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// To store the LiDAR pointcloud after icp has been applied
sensor_msgs::PointCloud2 transformed_cloud;
// To store the map pointcloud in as a ros msg type
sensor_msgs::PointCloud2 object_msg;
// To store the grid map pointcloud (not implemented yet)
sensor_msgs::PointCloud2 grid_msg;
// To store the residual error after icp for publishing
geometry_msgs::TwistStamped ResidualErrorMsg;

// To store the dimension of the pointcloud data
int cloudDimension;

// To store the path to config file where
// parameters for filters and icp are declared
string configFile = ros::package::getPath("libpointmatcher_ros");

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

// To store the map pointcloud
DP refer;
// To store the current LiDAR scan for icp
DP input_data;
// Performs icp on the pointclouds
PM::ICP icp;

tf::Transform performICP() {
  // Get dimension of input LiDAR pointcloud
  cloudDimension = input_data.getEuclideanDim();

  // Initializing data for ICP
  DP initializedData = input_data;

  // Performing the ICP and obtaining the transformation
  PM::TransformationParameters T = icp(initializedData, refer);

  // Copying initializedData to data_out for applying the transformation
  DP data_out(initializedData);
  icp.transformations.apply(data_out, T);

  // To compute residual error:
  // Setting reference cloud in the matcher
  icp.matcher->init(refer);
  // Getting matches between transformed data and ref
  PM::Matches matches = icp.matcher->findClosests(data_out);
  // Getting outlier weights for the matches
  PM::OutlierWeights outlierWeights =
      icp.outlierFilters.compute(data_out, refer, matches);
  // Computing error
  ResidualErrorMsg.twist.linear.x = icp.errorMinimizer->getResidualError(
      data_out, refer, outlierWeights, matches);

  // Adding header data to the message
  ResidualErrorMsg.header.stamp = ros::Time::now();
  ResidualErrorMsg.header.frame_id = "odom";

  // Display the transformation
  cout << "ICP transformation:" << endl << T << endl;

  // Copying correction_matrix into a tf::transform object
  tf::Vector3 origin;
  origin.setValue(static_cast<float>(T(0, 3)), static_cast<float>(T(1, 3)), 0);
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<float>(T(0, 0)), static_cast<float>(T(0, 1)),
                static_cast<float>(T(0, 2)), static_cast<float>(T(1, 0)),
                static_cast<float>(T(1, 1)), static_cast<float>(T(1, 2)),
                static_cast<float>(T(2, 0)), static_cast<float>(T(2, 1)),
                static_cast<float>(T(2, 2)));
  double r, p, y;
  tf3d.getRPY(r, p, y);
  tf3d.setRPY(0.0, 0.0, y);
  tf::Transform correction_transform;
  correction_transform.setOrigin(origin);
  correction_transform.setBasis(tf3d);

  // Convert transformed pointcloud to ros
  transformed_cloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
      data_out, "odom", ros::Time::now());
  transformed_cloud.header.frame_id = "odom";

  // Return the correction_transform to publish it
  return correction_transform;
}

void inputCloudSubscriber(
    const boost::shared_ptr<const sensor_msgs::PointCloud2> &input) {
  // Converting input cloud from ROS message to DataPoints format
  input_data = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*input);
}

int main(int argc, char **argv) {
  // Initialising the node
  ros::init(argc, argv, "ICP_on_map");
  ros::NodeHandle n;

  cout << endl << "LOADING MAP" << endl;

  // TODO: make this path an argument
  pcl::io::loadPCDFile<pcl::PointXYZ>(string(argv[1]), *map_cloud);
  ros::Subscriber sub =
      n.subscribe("/filtered_points", 1000, inputCloudSubscriber);
  ros::Publisher cloud_pub =
      n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);
  ros::Publisher error_pub =
      n.advertise<geometry_msgs::TwistStamped>("/ResidualError", 1000);

  // Converting from a pcl::PointCloud to DataPoint datatype
  pcl::toROSMsg(*map_cloud.get(), object_msg);
  refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(object_msg);

  // Getting configuration parameters
  configFile += "/params/filters.yaml";
  ifstream ifs(configFile.c_str());
  if (!ifs.good()) {
    cerr << "Cannot open config file " << configFile << ", usage:";
    exit(1);
  }

  // Loading ICP with the configuration specified in the file
  icp.loadFromYaml(ifs);

  // Getting dimension of the pointcloud and storing it in cloudDimension
  cloudDimension = refer.getEuclideanDim();
  if (!(cloudDimension == 2 || cloudDimension == 3)) {
    cerr << "Invalid input point clouds dimension" << endl;
    exit(1);
  }

  // Initial correction set to 0
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setIdentity();

  while (ros::ok()) {
    // Making sure input data makes sense
    if (input_data.getNbPoints() > 0) {
      // Sending correction transform
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
	  										"odom", "correction"));

      // Obtaining correction transform
      transform = performICP();

      // Publishing the transformed pointcloud
      cloud_pub.publish(transformed_cloud);

      // Publishing the error data
      error_pub.publish(ResidualErrorMsg);
    }
    ros::spinOnce();
  }
  return 0;
}
