// Author : Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include "ros/ros.h"
#include <bits/stdc++.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace pcl;

// Returns a transform from a stamped transform
tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf) {
  // Construct a transform using elements of sTf
  tf::Transform tf(sTf.getBasis(), sTf.getOrigin());
  return tf;
}

bool multiply_stamped_tfs(tf::StampedTransform t1_stf,
                          tf::StampedTransform t2_stf,
                          tf::StampedTransform &tmul_stf) {
  // Simple transforms--not stamped
  tf::Transform t1, t2, tmul;

  // Want to compare strings to check consistency
  std::string str1(t1_stf.child_frame_id_);
  std::string str2(t2_stf.frame_id_);

  // Get the transform from the stamped transform
  t1 = get_tf_from_stamped_tf(t1_stf);
  t2 = get_tf_from_stamped_tf(t2_stf);

  // Multiplication is defined for transforms
  tmul = t1 * t2;
  // Assign appropriate parent and child frames to result
  tmul_stf.frame_id_ = t1_stf.frame_id_;
  tmul_stf.child_frame_id_ = t2_stf.child_frame_id_;
  // Populate the origin and orientation of the result
  tmul_stf.setOrigin(tmul.getOrigin());
  tmul_stf.setBasis(tmul.getBasis());

  // Assign the time stamp to current time
  tmul_stf.stamp_ = ros::Time::now();

  return true;
}

int main(int argc, char **argv) {
  // Initializing the node
  ros::init(argc, argv, "correction_of_transform");
  ros::NodeHandle n;

  tf::TransformListener listener;
  static tf::TransformBroadcaster br;

  /*
		MERGING THE CORRECTIONS :

		BaseLink stores the transform of the bot had there been no ICP
		correction from the beginning 
		BaseLink transform is updated every frame by listening to the 
		base_link transform provided by the robot_localisation package

		CorrectionMatrix stores the latest correction transform provided by ICP 
		CorrectionMatrix is updated when a new correction is broadcasted after
		ICP by listening for a new correction transform every frame

        PrevCorrectionMatrix stores the value of the CorrectionMatrix in the
		previous frame 
		PrevCorrectionMatrix is updated every frame by inheriting the old 
		value of CorrectionMatrix. End of an ICP process (and start of a
		new ICP process) is deternmined by comparing PrevCorrectionMatrix to
		CorrectionMatrix.

        InverseBaseLink_A stores the inverse of the BaseLink transform when
		the latest ICP (incomplete) process started 
		InverseBaseLink_A is updated when a new ICP process starts 
		(and the previous process ends) by listening to the inverse of the 
		base_link transform at that moment

        InverseBaseLink_B stores the inverse of the BaseLink transform when
		the last completed ICP process had started 
		InverseBaseLink_B is updated when a new ICP process starts (and the 
		previous process ends) by inheriting the old value of InverseBaseLink_A

        CummTransformAfterLastICP stores the transformation that the bot has
		traversed since the last completed ICP process relative to the BaseLink
		transform at the moment when the last completed ICP had just started
        CummTransformAfterLastICP is updated every frame by multiplying
     	BaseLink and InverseBaseLink_B

        Corrected stores the final buest guess transform obtained by merging
		the corrections of ICP and the wheel odometry data from robot_localization
		package 
		Corrected is updated every frame by multiplying the LastCorrect with the 
		CummTransformAfterLastICP

		LastCorrect stores the last corrected transform that was directly
     	corrected by ICP 
		LastCorrect is updated when an ICP process ends (and a new one starts) by 
		multiplying the CorrectionMatrix with the InputForICP

        InputForICP stores input transform that went into the latest ICP process 
		InputForICP is updated when an ICP process starts (and the previous 
		process ends) by inheriting the value of Corrected at that moment
  */

  // Defining all the transforms
  tf::StampedTransform Corrected;
  tf::StampedTransform CorrectionMatrix;
  tf::StampedTransform PrevCorrectionMatrix;
  tf::StampedTransform BaseLink;
  tf::StampedTransform LastCorrect;
  tf::StampedTransform InverseBaseLink_A;
  tf::StampedTransform InverseBaseLink_B;
  tf::StampedTransform CummTransformAfterLastICP;
  tf::StampedTransform InputForICP;

  // Initializing all the transforms
  Corrected.setIdentity();
  CorrectionMatrix.setIdentity();
  PrevCorrectionMatrix.setIdentity();
  LastCorrect.setIdentity();
  InverseBaseLink_A.setIdentity();
  InverseBaseLink_B.setIdentity();
  CummTransformAfterLastICP.setIdentity();
  InputForICP.setIdentity();

  while (ros::ok()) {
    // Listening to the correction provided by ICP and updating CorrectionMatrix
    try {
      listener.lookupTransform("/odom", "correction", ros::Time(0),
                               CorrectionMatrix);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // Listening to the base_link transform provided by robot_localisation and
    // updating BaseLink
    try {
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), BaseLink);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // If there is a change of ICP processes
    if (CorrectionMatrix.stamp_ != PrevCorrectionMatrix.stamp_) {
      // Update LastCorrect
      multiply_stamped_tfs(CorrectionMatrix, InputForICP, LastCorrect);

      // Update InverseBaseLink_B
      InverseBaseLink_B = InverseBaseLink_A;

      // Update InverseBaseLink_A
      try {
        listener.lookupTransform("/base_link", "/odom", ros::Time(0),
                                 InverseBaseLink_A);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      // Update InputForICP
      InputForICP = Corrected;
    }

    // Update CummTransformAfterLastICP
    multiply_stamped_tfs(InverseBaseLink_B, BaseLink,
                         CummTransformAfterLastICP);
    // Update Corrected
    multiply_stamped_tfs(LastCorrect, CummTransformAfterLastICP, Corrected);

    // Broadcasting all relevant transforms
    br.sendTransform(tf::StampedTransform(Corrected, ros::Time::now(), 
										  "odom", "Corrected"));
    br.sendTransform(tf::StampedTransform(CorrectionMatrix, ros::Time::now(),
                                          "odom", "CorrectionMatrix"));
    br.sendTransform(tf::StampedTransform(LastCorrect, ros::Time::now(), 
									      "odom", "LastCorrect"));
    br.sendTransform(tf::StampedTransform(InputForICP, ros::Time::now(), 
										  "odom", "InputForICP_OLD"));
    br.sendTransform(tf::StampedTransform(CummTransformAfterLastICP,
                                          ros::Time::now(), "odom",
                                          "CummTransformAfterLastICP"));
    br.sendTransform(tf::StampedTransform(InverseBaseLink_B, ros::Time::now(),
                                          "base_link", "InverseBaseLink_B"));
    br.sendTransform(tf::StampedTransform(InverseBaseLink_A, ros::Time::now(),
                                          "base_link", "InverseBaseLink_A"));

    // Update PrevCorrectionMatrix
    PrevCorrectionMatrix = CorrectionMatrix;

    ros::spinOnce();
  }
  return (0);
}