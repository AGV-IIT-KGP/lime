//Author : Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include <ros/ros.h>
#include <boost/assign.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include "ros/time.h"
#include <bits/stdc++.h>

using namespace std;

// For storing the last error message encountered
geometry_msgs::TwistStamped prev_error_msg;
// For storing the path to the error file
string ErrorFilePath;

// Subscriber callback function
// Accepts geometry_msgs::Twist message containing encoder data
// Calculates the linear and angular velocity from encoder data. 
void errorCallback(geometry_msgs::TwistStamped error_msg)
{
	ofstream ResidualError;
	ResidualError.open (ErrorFilePath, ios::out | ios::app);
	ResidualError << endl << error_msg.header.stamp - prev_error_msg.header.stamp << "\t" << error_msg.twist.linear.x;
  	ResidualError.close();
	prev_error_msg = error_msg;
}

int main(int argc, char **argv)
{
	// Initialising the node
	ros::init(argc, argv, "error_saver");
	ros::NodeHandle n;

	ErrorFilePath = string(argv[1]);
	ofstream ResidualError;
	ResidualError.open (ErrorFilePath, ios::out | ios::trunc);	
	ResidualError.close();
	// Subscribing the encoder data on /encoders
	// Also calls callback function which calculates velocity data from encoders
	ros::Subscriber odom_sub = n.subscribe<geometry_msgs::TwistStamped>("/ResidualError", 50, errorCallback);

	while (ros::ok())
		ros::spin();

	return 0;
}