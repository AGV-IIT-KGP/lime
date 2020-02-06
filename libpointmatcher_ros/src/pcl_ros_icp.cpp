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
using namespace std;
using namespace pcl;

Eigen::Matrix4f cumm_transform;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
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


PM::TransformationParameters parseTranslation(string& translation,const int cloudDimension) 
{
	PM::TransformationParameters parsedTranslation;
	parsedTranslation = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);

	translation.erase(std::remove(translation.begin(), translation.end(), '['),translation.end());
	translation.erase(std::remove(translation.begin(), translation.end(), ']'),translation.end());
	std::replace( translation.begin(), translation.end(), ',', ' ');
	std::replace( translation.begin(), translation.end(), ';', ' ');

	float translationValues[3] = {0};
	stringstream translationStringStream(translation);
	for( int i = 0; i < cloudDimension; i++) {
		if(!(translationStringStream >> translationValues[i])) {
			cerr << "An error occured while trying to parse the initial "
				 << "translation." << endl
				 << "No initial translation will be used" << endl;
			return parsedTranslation;
		}
	}
	float extraOutput = 0;
	if((translationStringStream >> extraOutput)) {
		cerr << "Wrong initial translation size" << endl
			 << "No initial translation will be used" << endl;
		return parsedTranslation;
	}

	for( int i = 0; i < cloudDimension; i++) {
		parsedTranslation(i,cloudDimension) = translationValues[i];
	}

	return parsedTranslation;
}

PM::TransformationParameters parseRotation(string &rotation,const int cloudDimension)
{
	PM::TransformationParameters parsedRotation;
	parsedRotation = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);

	rotation.erase(std::remove(rotation.begin(), rotation.end(), '['),rotation.end());
	rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'),rotation.end());
	std::replace( rotation.begin(), rotation.end(), ',', ' ');
	std::replace( rotation.begin(), rotation.end(), ';', ' ');

	float rotationMatrix[9] = {0};
	stringstream rotationStringStream(rotation);
	for( int i = 0; i < cloudDimension*cloudDimension; i++) {
		if(!(rotationStringStream >> rotationMatrix[i])) {
			cerr << "An error occured while trying to parse the initial "
				 << "rotation." << endl
				 << "No initial rotation will be used" << endl;
			return parsedRotation;
		}
	}
	float extraOutput = 0;
	if((rotationStringStream >> extraOutput)) {
		cerr << "Wrong initial rotation size" << endl
			 << "No initial rotation will be used" << endl;
		return parsedRotation;
	}

	for( int i = 0; i < cloudDimension*cloudDimension; i++) {
		parsedRotation(i/cloudDimension,i%cloudDimension) = rotationMatrix[i];
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
	PM::TransformationParameters initTransfo = translation*rotation;
	
	std::shared_ptr<PM::Transformation> rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	if (!rigidTrans->checkParameters(initTransfo)) {
		cerr << endl
			 << "Initial transformation is not rigid, identiy will be used"
			 << endl;
		initTransfo = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);
	}

	DP initializedData = rigidTrans->compute(data, initTransfo);

	PM::TransformationParameters T = icp(initializedData, refer);
	cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

	DP data_out(initializedData);
	icp.transformations.apply(data_out, T);
		// data_out.save("test_data_out.pcd");
	cout << "ICP transformation:" << endl << T << endl;
	Eigen::Matrix4f transform_1;

	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			transform_1(i, j) = T(i, j);
		}
	}

    cout<<transform_1<<endl;
    cumm_transform=transform_1*cumm_transform;

    tf::Vector3 origin;
    origin.setValue(static_cast<float>(cumm_transform(0,3)),static_cast<float>(cumm_transform(1,3)), 0);//static_cast<float>(cumm_transform(2,3)));

    // cout << origin << endl;
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<float>(cumm_transform(0,0)), static_cast<float>(cumm_transform(0,1)), static_cast<float>(cumm_transform(0,2)), 
            static_cast<float>(cumm_transform(1,0)), static_cast<float>(cumm_transform(1,1)), static_cast<float>(cumm_transform(1,2)), 
            static_cast<float>(cumm_transform(2,0)), static_cast<float>(cumm_transform(2,1)), static_cast<float>(cumm_transform(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);
       
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "cumm"));
    transformed_cloud=PointMatcher_ros::pointMatcherCloudToRosMsg<float>(initializedData,"odom",ros::Time::now()); 
	transformed_cloud.header.frame_id = "odom";
    // pcl::transformPointCloud (*current_lidar, *transformed_cloud, transform_1);
}

void pc2_to_pcl_plus_icp(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
	data = PointMatcher_ros::rosMsgToPointMatcherCloud<float> (*input);
}
int main (int argc, char** argv)
{
	ros::init(argc, argv, "ICP_on_map");
	ros::NodeHandle n;
    cout<<endl<<"adada"<<endl;
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/high res map/kitti final map.pcd", *cloud_out);
	ros::Subscriber sub = n.subscribe("/shifted_points", 1000, pc2_to_pcl_plus_icp);
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);

    pcl::toROSMsg(*cloud_out.get(),object_msg);
    refer = PointMatcher_ros::rosMsgToPointMatcherCloud<float> (object_msg);

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


    cumm_transform << 1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1;

    tf::Vector3 origin;
    origin.setValue(static_cast<float>(cumm_transform(0,3)),static_cast<float>(cumm_transform(1,3)), static_cast<float>(cumm_transform(2,3)));

    cout << origin << endl;
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<float>(cumm_transform(0,0)), static_cast<float>(cumm_transform(0,1)), static_cast<float>(cumm_transform(0,2)), 
            static_cast<float>(cumm_transform(1,0)), static_cast<float>(cumm_transform(1,1)), static_cast<float>(cumm_transform(1,2)), 
            static_cast<float>(cumm_transform(2,0)), static_cast<float>(cumm_transform(2,1)), static_cast<float>(cumm_transform(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "cumm"));   
    while (ros::ok())
    {
        //sensor_msgs::PointCloud2 object_msg;
        do_pcl();
        //pcl::toROSMsg(*transformed_cloud.get(),object_msg );
        //transformed_cloud.header.frame_id = "odom";
        cloud_pub.publish(transformed_cloud);
	    ros::spinOnce();
    }
	return 0;
}

