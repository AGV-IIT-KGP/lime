#include <iostream>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
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
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace pcl;

tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf) {
   tf::Transform tf(sTf.getBasis(),sTf.getOrigin()); //construct a transform using elements of sTf
   return tf;
}

bool multiply_stamped_tfs(tf::StampedTransform A_stf, tf::StampedTransform B_stf, tf::StampedTransform &C_stf)
{
	tf::Transform A,B,C; //simple transforms--not stamped
	std::string str1 (A_stf.child_frame_id_); //want to compare strings to check consistency
	std::string str2 (B_stf.frame_id_);

	//if here, the named frames are logically consistent
	A = get_tf_from_stamped_tf(A_stf); // get the transform from the stamped transform
	B = get_tf_from_stamped_tf(B_stf);
	C = A*B; //multiplication is defined for transforms 
	C_stf.frame_id_ = A_stf.frame_id_; //assign appropriate parent and child frames to result
	C_stf.child_frame_id_ = B_stf.child_frame_id_;
	C_stf.setOrigin(C.getOrigin()); //populate the origin and orientation of the result
	C_stf.setBasis(C.getBasis());
	C_stf.stamp_ = ros::Time::now(); //assign the time stamp to current time; 
		// alternatively, could assign this to the OLDER of A or B transforms
return true; //if got here, the multiplication is valid
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "correction_of_transform");
	ros::NodeHandle n;
	tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    
	tf::StampedTransform Corrected;
	tf::StampedTransform CorrectionMatrix;
	tf::StampedTransform PrevCorrectionMatrix;
	tf::StampedTransform BaseLink;
	tf::StampedTransform LastCorrect;
	tf::StampedTransform InverseBaseLink_A;
	tf::StampedTransform InverseBaseLink_B;
	tf::StampedTransform CummTransformAfterLastICP;
	tf::StampedTransform CorrectedForICP;


	Corrected.setIdentity();
	CorrectionMatrix.setIdentity();
	PrevCorrectionMatrix.setIdentity();
	LastCorrect.setIdentity();
	InverseBaseLink_A.setIdentity();
	InverseBaseLink_B.setIdentity();
	CummTransformAfterLastICP.setIdentity();
	CorrectedForICP.setIdentity();

	while (ros::ok())
    {
        try
		{
			listener.lookupTransform("/odom", "correction",  ros::Time(0), CorrectionMatrix);
		}
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

		try
		{
			listener.lookupTransform("/odom", "/base_link",  ros::Time(0), BaseLink);
		}
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

		if (CorrectionMatrix.stamp_ != PrevCorrectionMatrix.stamp_)
		{
			multiply_stamped_tfs (CorrectionMatrix, CorrectedForICP, LastCorrect);
			
			InverseBaseLink_B = InverseBaseLink_A;
			
			try
			{
				listener.lookupTransform("/base_link", "/odom",  ros::Time(0), InverseBaseLink_A);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}

		if (CorrectionMatrix.stamp_ != PrevCorrectionMatrix.stamp_)
		{
			// try
			// {
			// 	listener.lookupTransform("/odom", "CorrectedForICP",  ros::Time(0), CorrectedForICP);
			// }
			// catch (tf::TransformException ex)
			// {
			// 	ROS_ERROR("%s",ex.what());
			// 	ros::Duration(1.0).sleep();
			// }
			CorrectedForICP = Corrected;
		}
		
		multiply_stamped_tfs(InverseBaseLink_B, BaseLink, CummTransformAfterLastICP);
		multiply_stamped_tfs(LastCorrect, CummTransformAfterLastICP, Corrected);
		

        br.sendTransform(tf::StampedTransform(Corrected, ros::Time::now(), "odom", "Corrected"));
		br.sendTransform(tf::StampedTransform(CorrectionMatrix, ros::Time::now(), "odom", "CorrectionMatrix"));
		br.sendTransform(tf::StampedTransform(LastCorrect, ros::Time::now(), "odom", "LastCorrect"));
		br.sendTransform(tf::StampedTransform(CorrectedForICP, ros::Time::now(), "odom", "CorrectedForICP_OLD"));
		// br.sendTransform(tf::StampedTransform(CummTransformAfterLastICP, ros::Time::now(), "LastCorrect", "cummtransform"));
		br.sendTransform(tf::StampedTransform(CummTransformAfterLastICP, ros::Time::now(), "odom", "CummTransformAfterLastICP"));
		br.sendTransform(tf::StampedTransform(InverseBaseLink_B, ros::Time::now(), "base_link", "InverseBaseLink_B"));
		br.sendTransform(tf::StampedTransform(InverseBaseLink_A, ros::Time::now(), "base_link", "InverseBaseLink_A"));	
	
		PrevCorrectionMatrix = CorrectionMatrix;
	    ros::spinOnce();
    }
    return(0);
}