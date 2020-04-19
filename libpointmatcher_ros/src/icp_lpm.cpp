#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Bibliography.h"
#include "pointmatcher_ros/point_cloud.h"
#include "boost/filesystem.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
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

//#include <eigen3/Eigen>
#include <pcl/filters/extract_indices.h>

#define TILE_X 40
#define TILE_Y 40

// Todo: check grid mapping
// Todo: check LastCorrect authenticity

using namespace std;
using namespace pcl;

Eigen::Matrix4f correction_matrix;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 transformed_cloud;
sensor_msgs::PointCloud2 object_msg;
int cloudDimension;
string configFile = "/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/libpointmatcher_ros/params/filters.yaml";

string initTranslation("0,0,0");
string initRotation("1,0,0;0,1,0;0,0,1");

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

DP refer;
DP data;
PM::ICP icp;

// tf::TransformListener listener;
// tf::StampedTransform CorrectedForICP;

PM::TransformationParameters parseTranslation(string &translation, const int cloudDimension)
{
	PM::TransformationParameters parsedTranslation;
	parsedTranslation = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);

	translation.erase(std::remove(translation.begin(), translation.end(), '['), translation.end());
	translation.erase(std::remove(translation.begin(), translation.end(), ']'), translation.end());
	std::replace(translation.begin(), translation.end(), ',', ' ');
	std::replace(translation.begin(), translation.end(), ';', ' ');

	float translationValues[3] = {0};
	stringstream translationStringStream(translation);
	for (int i = 0; i < cloudDimension; i++)
	{
		if (!(translationStringStream >> translationValues[i]))
		{
			cerr << "An error occured while trying to parse the initial "
				 << "translation." << endl
				 << "No initial translation will be used" << endl;
			return parsedTranslation;
		}
	}
	float extraOutput = 0;
	if ((translationStringStream >> extraOutput))
	{
		cerr << "Wrong initial translation size" << endl
			 << "No initial translation will be used" << endl;
		return parsedTranslation;
	}

	for (int i = 0; i < cloudDimension; i++)
	{
		parsedTranslation(i, cloudDimension) = translationValues[i];
	}

	return parsedTranslation;
}

PM::TransformationParameters parseRotation(string &rotation, const int cloudDimension)
{
	PM::TransformationParameters parsedRotation;
	parsedRotation = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);

	rotation.erase(std::remove(rotation.begin(), rotation.end(), '['), rotation.end());
	rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'), rotation.end());
	std::replace(rotation.begin(), rotation.end(), ',', ' ');
	std::replace(rotation.begin(), rotation.end(), ';', ' ');

	float rotationMatrix[9] = {0};
	stringstream rotationStringStream(rotation);
	for (int i = 0; i < cloudDimension * cloudDimension; i++)
	{
		if (!(rotationStringStream >> rotationMatrix[i]))
		{
			cerr << "An error occured while trying to parse the initial "
				 << "rotation." << endl
				 << "No initial rotation will be used" << endl;
			return parsedRotation;
		}
	}
	float extraOutput = 0;
	if ((rotationStringStream >> extraOutput))
	{
		cerr << "Wrong initial rotation size" << endl
			 << "No initial rotation will be used" << endl;
		return parsedRotation;
	}

	for (int i = 0; i < cloudDimension * cloudDimension; i++)
	{
		parsedRotation(i / cloudDimension, i % cloudDimension) = rotationMatrix[i];
	}

	return parsedRotation;
}

void do_pcl()
{
	if (data.getNbPoints() <= 0)
	{
		return;
	}

	static tf::TransformBroadcaster br;

	cloudDimension = data.getEuclideanDim();

	PM::TransformationParameters translation = parseTranslation(initTranslation, cloudDimension);
	PM::TransformationParameters rotation = parseRotation(initRotation, cloudDimension);
	PM::TransformationParameters initTransfo = translation * rotation;

	std::shared_ptr<PM::Transformation> rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	if (!rigidTrans->checkParameters(initTransfo))
	{
		cerr << endl
			 << "Initial transformation is not rigid, identiy will be used"
			 << endl;
		initTransfo = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);
	}

	DP initializedData = rigidTrans->compute(data, initTransfo);

	PM::TransformationParameters T = icp(initializedData, refer);
	cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

	DP data_out(initializedData);
	icp.transformations.apply(data_out, T);
	// data_out.save("test_data_out.pcd");
	cout << "ICP transformation:" << endl
		 << T << endl;
	Eigen::Matrix4f transform_1;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			transform_1(i, j) = T(i, j);
		}
	}

	cout << transform_1 << endl;
	correction_matrix = transform_1;
	

	tf::Vector3 origin;
	origin.setValue(static_cast<float>(correction_matrix(0, 3)), static_cast<float>(correction_matrix(1, 3)), 0); //static_cast<float>(correction_matrix(2,3)));

	// cout << origin << endl;
	tf::Matrix3x3 tf3d;
	tf3d.setValue(static_cast<float>(correction_matrix(0, 0)), static_cast<float>(correction_matrix(0, 1)), static_cast<float>(correction_matrix(0, 2)),
				  static_cast<float>(correction_matrix(1, 0)), static_cast<float>(correction_matrix(1, 1)), static_cast<float>(correction_matrix(1, 2)),
				  static_cast<float>(correction_matrix(2, 0)), static_cast<float>(correction_matrix(2, 1)), static_cast<float>(correction_matrix(2, 2)));

	tf::Quaternion tfqt;
	double r, p, y;
	tf3d.getRPY(r, p, y);
	tf3d.setRPY (0.0, 0.0, y);
	tf3d.getRotation(tfqt);
	tf::Transform transform;
	transform.setOrigin(origin);
	transform.setRotation(tfqt);

	// br.sendTransform(tf::StampedTransform(CorrectedForICP, ros::Time::now(), "odom", "CorrectedForICP"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "correction"));
	if (initializedData.getNbPoints() <= 0)
	{
		return;
	}
	transformed_cloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(initializedData, "odom", ros::Time::now());
	transformed_cloud.header.frame_id = "odom";
	// pcl::transformPointCloud (*current_lidar, *transformed_cloud, transform_1);
}

void get_map(int &N1, int &N2, int &N1_prev, int &N2_prev)
{
	tf::StampedTransform transform1;
	tf::StampedTransform transform2;
	tf::TransformListener listener;

	try
	{
		listener.lookupTransform("odom", "base_link", ros::Time(0), transform1);
		//cout<<transform.translation.x<<endl<<transform.translation.y<<endl<<endl;
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	tf::Matrix3x3 m(transform1.getRotation());
	double r, p, y;
	m.getRPY(r, p, y);
	r = 0;
	p = 0;
	Eigen::Matrix4f Tm, Rm, CummTransform;
	// CummTransform << 1, 0, 0, 0,
	//     0, 1, 0, 0,
	//     0, 0, 1, 0,
	//     0, 0, 0, 1;
	// Rm << 1, 0, 0, 0,
	//     0, 1, 0, 0,
	//     0, 0, 1, 0,
	//     0, 0, 0, 1;
	Rm << 1, 0, 0, transform1.getOrigin().x(),
		0, 1, 0, transform1.getOrigin().y(),
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
	// pcl_ros::transformPointCloud (Rm, pcin, pcout);
	// pcl_ros::transformPointCloud (Tm, pcout, pcout2);

	// CummTransform = Tm * (Rm * CummTransform);

	// try
	// {
	//   listener.lookupTransform("base_link", "cumm", ros::Time(0), transform2);
	//   //cout<<transform.translation.x<<endl<<transform.translation.y<<endl<<endl;
	// }
	// catch (tf::TransformException ex)
	// {
	//   ROS_ERROR("%s", ex.what());
	//   ros::Duration(1.0).sleep();
	// }
	// Eigen::Matrix4f Tm1, Rm1;
	// m = tf::Matrix3x3(transform2.getRotation());
	// //double r,p,y;
	// m.getRPY(r, p, y);
	// r = 0;
	// p = 0;
	// Eigen::Matrix4f Tm, Rm;
	// Rm1 << 1, 0, 0, 0,
	//     0, 1, 0, 0,
	//     0, 0, 1, 0,
	//     0, 0, 0, 1;
	// Rm1 << 1, 0, 0, transform2.getOrigin().x(),
	//     0, 1, 0, transform2.getOrigin().y(),
	//     0, 0, 1, 0, //transform2.getOrigin().z(),
	//     0, 0, 0, 1;
	// Rm1(0, 0) = cos(y) * cos(p);
	// Rm1(0, 1) = (cos(y) * sin(p) * sin(r)) - (sin(y) * cos(r));
	// Rm1(0, 2) = (cos(y) * sin(p) * cos(r)) + (sin(y) * sin(r));
	// Rm1(1, 0) = sin(y) * cos(p);
	// Rm1(1, 1) = (sin(y) * sin(p) * sin(r)) + (cos(y) * cos(r));
	// Rm1(1, 2) = (sin(y) * sin(p) * cos(r)) - (cos(y) * sin(r));
	// Rm1(2, 0) = -sin(p);
	// Rm1(2, 1) = cos(p) * sin(r);
	// Rm1(2, 2) = cos(p) * cos(r);

	// Tm1 =  Rm;

	int X_car = Rm(0, 3);
	int Y_car = Rm(1, 3);

	// cout<<X_car<<" "<<Y_car<<endl<<Tm<<endl;

	N1 = X_car / TILE_X;
	N2 = Y_car / TILE_Y;

	if (N1_prev != N1 || N2_prev != N2)
	{
		(*cloud_out).clear();
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
					(*cloud_out) += tempCloud;
				}
			}
		}
	}
	// loadNewMap();
	if (N1_prev != N1 || N2_prev != N2)
	{
		sensor_msgs::PointCloud2 object_msg_new;
		pcl::toROSMsg(*cloud_out, object_msg_new);
		refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(object_msg_new);
	}
}

void pc2_to_pcl_plus_icp(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	data = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*input);
	
	// try
	// {
	// 	listener.lookupTransform("odom", "Corrected",  ros::Time(0), CorrectedForICP);
	// }
	// catch (tf::TransformException ex)
	// {
	// 	ROS_ERROR("%s",ex.what());
	// 	ros::Duration(1.0).sleep();
	// }
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ICP_on_map");
	ros::NodeHandle n;
	cout << endl
		 << "adada" << endl;

	// CorrectedForICP.setIdentity();
	
	pcl::io::loadPCDFile<pcl::PointXYZ>("/media/shreyanshdarshan/New Volume/vision/PCL/XYZ2PCD/build/pepsi_down.pcd", *cloud_out);
	ros::Subscriber sub = n.subscribe("/shifted_points", 1000, pc2_to_pcl_plus_icp);
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);
	ros::Publisher grid_pub = n.advertise<sensor_msgs::PointCloud2>("/grid_map", 1000);

	int N1 = 0, N2 = 0, N1_prev = 0, N2_prev = 0;

	pcl::toROSMsg(*cloud_out.get(), object_msg);
	refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(object_msg);

	ifstream ifs(configFile.c_str());
	if (!ifs.good())
	{
		cerr << "Cannot open config file " << configFile << ", usage:";
		exit(1);
	}
	icp.loadFromYaml(ifs);

	cloudDimension = refer.getEuclideanDim();

	if (!(cloudDimension == 2 || cloudDimension == 3))
	{
		cerr << "Invalid input point clouds dimension" << endl;
		exit(1);
	}

	correction_matrix << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	tf::Vector3 origin;
	origin.setValue(static_cast<float>(correction_matrix(0, 3)), static_cast<float>(correction_matrix(1, 3)), static_cast<float>(correction_matrix(2, 3)));

	cout << origin << endl;
	tf::Matrix3x3 tf3d;
	tf3d.setValue(static_cast<float>(correction_matrix(0, 0)), static_cast<float>(correction_matrix(0, 1)), static_cast<float>(correction_matrix(0, 2)),
				  static_cast<float>(correction_matrix(1, 0)), static_cast<float>(correction_matrix(1, 1)), static_cast<float>(correction_matrix(1, 2)),
				  static_cast<float>(correction_matrix(2, 0)), static_cast<float>(correction_matrix(2, 1)), static_cast<float>(correction_matrix(2, 2)));

	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);

	tf::Transform transform;
	transform.setOrigin(origin);
	transform.setRotation(tfqt);
	static tf::TransformBroadcaster br;

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "correction"));
	while (ros::ok())
	{
		//get_map(N1, N2, N1_prev, N2_prev);
		// if (N1_prev != N1 || N2_prev != N2)
		// {
		// 	cout<<"11111111111111111111111111111111111111"<<endl<<endl;
		// 	refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float> (object_msg);
		// 	cloud_pub.publish(transformed_cloud);
		// }
		// N1_prev = N1;
		// N2_prev = N2;
		//sensor_msgs::PointCloud2 object_msg;
		do_pcl();
		//pcl::toROSMsg(*transformed_cloud.get(),object_msg );
		//transformed_cloud.header.frame_id = "odom";
		cloud_pub.publish(transformed_cloud);
		ros::spinOnce();
	}
	return 0;
}
