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

void get_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	pcin = *input;
	return;
}
int main(int argc, char **argv)
{
	tf::StampedTransform transform1;
	tf::StampedTransform transform2;

	// Initializing the node
	ros::init(argc, argv, "shift_for_ICP");
	ros::NodeHandle node;
	ros::Rate rate(1000.0);
	tf::TransformListener listener;

	// Subscriber callback
	// Retrieves LiDAR point cloud from the /velodyne_points topic (frame_id = velodyne_points)
	ros::Subscriber sub = node.subscribe("/velodyne_points", 1000, get_cloud);

	ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/shifted_points", 1000);

	// Z value for ground thresholding
	// float zAvg = -1.45;

	// Roll and Pitch offset for manual correction for slightly
	float rolloff = -0.08, pitchoff = -0.045;

	int Frame = 0;
	while (node.ok())
	{

		/* 	This code helps debug the correction maintainer by sending a fake correction every 200 frames
			TODO: delete this section for final release of package
		
			Frame++;
			cout<<"Frame: "<<Frame<<endl;
			if (Frame > 200)
			{
				Frame = 0;

				tf::Quaternion yawer;
				tf::Vector3 orign;
				orign.setZero();
				yawer.setRPY(0, 0, 0.01);
				tf::Transform corr;
				corr.setRotation(yawer);
				corr.setOrigin(orign);
				// static tf::TransformBroadcaster br;
				// br.sendTransform(tf::StampedTransform(corr, ros::Time::now(), "/odom", "correction"));

				cout << endl
					<< "corrected" << endl;
			}

		*/

		// Get transform of Corrected frame wrt odom
		try
		{
			listener.lookupTransform("odom", "Corrected", ros::Time(0), transform1);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		// Make a 3x3 matrix containing the rotation information of the Corrected transform
		tf::Matrix3x3 m(transform1.getRotation());

		// Gets roll, pitch and yaw of the rotation matrix
		double r, p, y;
		m.getRPY(r, p, y);
		// Override the roll and pitch data with the manual corrections
		// Considering a 3dof localisation, roll and pitch matching are insignificant and sometimes create divergence
		r = rolloff;
		p = pitchoff;
		m.setRPY(r, p, y - 3.14159 / 2);

		/* TODO: check if the transformPointCloud function works properly and delete this commented section if it does
		
			Eigen::Matrix4f Tm, Rm;
			Rm << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
			Tm << 0, 1, 0, transform1.getOrigin().x(),
				-1, 0, 0, transform1.getOrigin().y(),
				0, 0, 1, 0, //transform1.getOrigin().z(),
				0, 0, 0, 1;
			Rm(0, 0) = cos(y) * cos(p);
			Rm(0, 1) = (cos(y) * sin(p) * sin(r)) - (sin(y) * cos(r));
			Rm(0, 2) = (cos(y) * sin(p) * cos(r)) + (sin(y) * sin(r));
			Rm(1, 0) = sin(y) * cos(p);
			Rm(1, 1) = (sin(y) * sin(p) * sin(r)) + (cos(y) * cos(r));
			Rm(1, 2) = (sin(y) * sin(p) * cos(r)) - (cos(y) * sin(r));
			Rm(2, 0) = -sin(p);
			Rm(2, 1) = cos(p) * sin(r);
			Rm(2, 2) = cos(p) * cos(r);
			//std::cout<<Rm<<std::endl;
			// pcl_ros::transformPointCloud(Rm, pcin, pcout);
			// pcl_ros::transformPointCloud(Tm, pcout, pcout2);
		
		*/

		// Overriding the transforms rotation with the corrected roll, pitch and yaw
		tf::Quaternion tfqt;
		m.getRotation(tfqt);
		transform1.setRotation(tfqt);

		// Transform the point cloud with the
		pcl_ros::transformPointCloud("odom", get_tf_from_stamped_tf(transform1), pcin, pcout);

		// // Ground removal from the 
		// pcl::PCLPointCloud2 groundless;
		// pcl_conversions::toPCL(pcout, groundless);

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
		// 	pcl::toROSMsg(*p_obstacles.get(), pcout);
		// }

		// Publish final transformed and ground removed cloud in odom frame
		pcout.header.frame_id = "odom";
		cloud_pub.publish(pcout);

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}