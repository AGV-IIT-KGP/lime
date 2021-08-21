//Author : Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

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
sensor_msgs::PointCloud2 pcout;

// Returns the tf::Transform equivalent of a tf::StampedTransform
tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf)
{
	// Construct a transform using elements of sTf
	tf::Transform tf(sTf.getBasis(), sTf.getOrigin());
	return tf;
}

// Subscriber for input pointcloud
void get_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	pcin = *input;
	return;
}
int main(int argc, char **argv)
{
	tf::StampedTransform transform;

	// Initializing the node
	ros::init(argc, argv, "shift_for_ICP");
	ros::NodeHandle node;
	ros::Rate rate(1000.0);
	tf::TransformListener listener;

	// Subscriber callback
	// Retrieves LiDAR point cloud from the /velodyne_points topic (frame_id = velodyne_points)
	ros::Subscriber sub = node.subscribe("/velodyne_points", 1000, get_cloud);

	ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/shifted_points", 1000);

	// Roll and Pitch offset for manual correction for slightly
	float rolloff = -0.08, pitchoff = -0.045;

	int Frame = 0;
	while (node.ok())
	{
		// Get transform of Corrected frame wrt odom
		try
		{
			listener.lookupTransform("odom", "Corrected", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		// Make a 3x3 matrix containing the rotation information of the Corrected transform
		tf::Matrix3x3 m(transform.getRotation());

		// Gets roll, pitch and yaw of the rotation matrix
		double r, p, y;
		m.getRPY(r, p, y);
		// Override the roll and pitch data with the manual corrections
		// Considering a 3dof localisation, roll and pitch matching are insignificant and sometimes create divergence
		r = rolloff;
		p = pitchoff;
		m.setRPY(r, p, y - 3.14159 / 2);

		// Overriding the transforms rotation with the corrected roll, pitch and yaw
		tf::Quaternion tfqt;
		m.getRotation(tfqt);
		transform.setRotation(tfqt);

		// Transform the point cloud with the
		pcl_ros::transformPointCloud("odom", get_tf_from_stamped_tf(transform), pcin, pcout);

		// Publish final transformed and ground removed cloud in odom frame
		pcout.header.frame_id = "odom";
		cloud_pub.publish(pcout);

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}