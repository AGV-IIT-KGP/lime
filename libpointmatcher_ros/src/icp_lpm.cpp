// Author: Radhika Patwawri, Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Bibliography.h"
#include "pointmatcher_ros/point_cloud.h"
#include "boost/filesystem.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bits/stdc++.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/filters/extract_indices.h>

#define TILE_X 40
#define TILE_Y 40

// Todo: grid map not implemented yet. functino: get_map()

using namespace std;
using namespace pcl;

// To store the map pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

// TODO: make paths machine independent
// To store the path to config file where parameters for filters and icp are declared
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

tf::Transform performICP()
{
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
	PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(data_out, refer, matches);
	// Computing error
	ResidualErrorMsg.twist.linear.x = icp.errorMinimizer->getResidualError(data_out, refer, outlierWeights, matches);

	// Adding header data to the message
	ResidualErrorMsg.header.stamp = ros::Time::now();
	ResidualErrorMsg.header.frame_id = "odom";

	// Display the transformation
	cout << "ICP transformation:" << endl
		 << T << endl;


	// Copying correction_matrix into a tf::transform object
	tf::Vector3 origin;
	origin.setValue(static_cast<float>(T(0, 3)), static_cast<float>(T(1, 3)), 0); //static_cast<float>(T(2,3)));
	tf::Matrix3x3 tf3d;
	tf3d.setValue(static_cast<float>(T(0, 0)), static_cast<float>(T(0, 1)), static_cast<float>(T(0, 2)),
				  static_cast<float>(T(1, 0)), static_cast<float>(T(1, 1)), static_cast<float>(T(1, 2)),
				  static_cast<float>(T(2, 0)), static_cast<float>(T(2, 1)), static_cast<float>(T(2, 2)));
	double r, p, y;
	tf3d.getRPY(r, p, y);
	tf3d.setRPY(0.0, 0.0, y);
	tf::Transform correction_transform;
	correction_transform.setOrigin(origin);
	correction_transform.setBasis(tf3d);

	// Convert transformed pointcloud to ros
	transformed_cloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_out, "odom", ros::Time::now());
	transformed_cloud.header.frame_id = "odom";

	// Return the correction_transform to publish it
	return correction_transform;
}

void get_map(int &N1, int &N2, int &N1_prev, int &N2_prev)
{
	static tf::TransformListener listener;
	tf::StampedTransform Bot_transform;

	try
	{
		listener.lookupTransform("odom", "Corrected", ros::Time(0), Bot_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	N1 = Bot_transform.getOrigin().x() / TILE_X;
	N2 = Bot_transform.getOrigin().y() / TILE_Y;

	if (N1_prev != N1 || N2_prev != N2)
	{
		(*map_cloud).clear();
		cout << N1 << " " << N2 << endl;
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				string path = "/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/grid_map/";
				string i_str = to_string(i + N1);
				string j_str = to_string(j + N2);
				path += i_str;
				path += "_";
				path += j_str;
				path += ".pcd";
				pcl::PointCloud<pcl::PointXYZ> tempCloud;
				if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, tempCloud) != -1)
				{
					(*map_cloud) += tempCloud;
				}
			}
		}
	}
	
	if (N1_prev != N1 || N2_prev != N2)
	{
		pcl::toROSMsg(*map_cloud, grid_msg);
		refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(grid_msg);
	}
}

void pc2_to_pcl_plus_icp(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	// Converting input cloud from ROS message to DataPoints format 
	input_data = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*input);
}

int main(int argc, char **argv)
{
	// Initialising the node
	ros::init(argc, argv, "ICP_on_map");
	ros::NodeHandle n;

	cout<< endl << "LOADING MAP" << endl;

	// TODO: make this path an argument
	pcl::io::loadPCDFile<pcl::PointXYZ>("/media/shreyanshdarshan/New Volume/vision/PCL/XYZ2PCD/build/pepsi_down.pcd", *map_cloud);
	ros::Subscriber sub = n.subscribe("/filtered_points", 1000, pc2_to_pcl_plus_icp);
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);
	ros::Publisher grid_pub = n.advertise<sensor_msgs::PointCloud2>("/grid_map", 1000);
	ros::Publisher error_pub = n.advertise<geometry_msgs::TwistStamped>("/ResidualError", 1000);

	// TODO: implement get_map() correctly
	int N1 = 0, N2 = 0, N1_prev = 0, N2_prev = 0;

	// Converting from a pcl::PointCloud to DataPoint datatype
	pcl::toROSMsg(*map_cloud.get(), object_msg);
	refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(object_msg);

	// Getting configuration parameters
	configFile += "/params/filters.yaml";
	ifstream ifs(configFile.c_str());
	if (!ifs.good())
	{
		cerr << "Cannot open config file " << configFile << ", usage:";
		exit(1);
	}

	// Loading ICP with the configuration specified in the file
	icp.loadFromYaml(ifs);

	// Getting dimension of the pointcloud and storing it in cloudDimension
	cloudDimension = refer.getEuclideanDim();
	if (!(cloudDimension == 2 || cloudDimension == 3))
	{
		cerr << "Invalid input point clouds dimension" << endl;
		exit(1);
	}

	// Initial correction set to 0
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setIdentity();

	// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "correction"));
	while (ros::ok())
	{
		// Making sure input data makes sense
		if (input_data.getNbPoints() > 0)
		{
			// TODO: Implement get_map() correctly
			// get_map(N1, N2, N1_prev, N2_prev);

			// Sending correction transform
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "correction"));
			
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
