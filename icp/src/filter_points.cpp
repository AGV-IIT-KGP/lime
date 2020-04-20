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

void get_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	// sensor_msgs::PointCloud2 pcin = *input;
	// sensor_msgs::convertPointCloud2ToPointCloud (temp, pcin);
	pcin = *input;
	return;
}

sensor_msgs::PointCloud2 removeGround(sensor_msgs::PointCloud2 input_cloud, int z_thresh)
{
	sensor_msgs::PointCloud2 output_cloud;
	pcl::PCLPointCloud2 groundless;
	pcl_conversions::toPCL(input_cloud, groundless);
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundless_P(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(groundless, *groundless_P);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	for (int i = 0; i < (*groundless_P).size(); i++)
	{
		pcl::PointXYZ pt(groundless_P->points[i].x, groundless_P->points[i].y, groundless_P->points[i].z);
		if (z_thresh - pt.z > 0) // e.g. remove all pts below zAvg
		{
			inliers->indices.push_back(i);
		}
	}
	if ((*groundless_P).size() > 0)
	{
		extract.setInputCloud(groundless_P);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*groundless_P);
		pcl::toROSMsg(*groundless_P.get(), output_cloud);
	}

	output_cloud.header.frame_id = "odom";
	return output_cloud;
}

int main(int argc, char **argv)
{
	tf::StampedTransform transform1;
	ros::init(argc, argv, "Filter_Cloud");

	ros::NodeHandle node;

	ros::Rate rate(1000.0);

	tf::TransformListener listener;
	ros::Subscriber sub = node.subscribe("/shifted_points", 1000, get_cloud);
	ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1000);

	float zAvg = -1.6;

	while (node.ok())
	{
		// pcl::PCLPointCloud2 groundless;
		// pcl_conversions::toPCL(pcin, groundless);

		// pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::fromPCLPointCloud2(groundless, *p_obstacles);
		// pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		// pcl::ExtractIndices<pcl::PointXYZ> extract;

		// for (int i = 0; i < (*p_obstacles).size(); i++)
		// {
		// 	pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y, p_obstacles->points[i].z);
		// 	if (zAvg - pt.z > 0) // e.g. remove all pts below zAvg
		// 	{
		// 		inliers->indices.push_back(i);
		// 	}
		// }
		// if ((*p_obstacles).size() > 0)
		// {
		// 	extract.setInputCloud(p_obstacles);
		// 	extract.setIndices(inliers);
		// 	extract.setNegative(true);
		// 	extract.filter(*p_obstacles);
		// 	pcl::toROSMsg(*p_obstacles.get(), pcin);
		// }

		// pcin.header.frame_id = "odom";

		// cout << transform1.getOrigin().x() << endl;

		cloud_pub.publish(removeGround(pcin, zAvg));
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}