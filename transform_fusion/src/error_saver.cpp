//Author : Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include <ros/ros.h>
#include <boost/assign.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include "ros/time.h"
#include <iostream>
#include <fstream>

#define fac 1.5
#define speed 1

using namespace std;

// For storing calculated velocity data before publishing
geometry_msgs::TwistStamped velocity_msg;

// vl and vr will store left and right wheel odometry information
// r will store the turning radius calculated from vl and vr
// dist is the distance between the wheels of the vehicle
float vl, vr, r;
float dist = 1.45;

// For publishing the calculated velocity data
ros::Publisher odom_pub;
geometry_msgs::TwistStamped prev_error_msg;

// Subscriber callback function
// Accepts geometry_msgs::Twist message containing encoder data
// Calculates the linear and angular velocity from encoder data. 
void errorCallback(geometry_msgs::TwistStamped error_msg)
{
	ofstream ResidualError;
	ResidualError.open ("/home/shreyanshdarshan/Localization/catkin_ws/ResidualError.txt", ios::out | ios::app);
	ResidualError << endl << error_msg.header.stamp - prev_error_msg.header.stamp << "\t" << error_msg.twist.linear.x;
  	ResidualError.close();
	prev_error_msg = error_msg;
}

int main(int argc, char **argv)
{
	// Initialising the node
	ros::init(argc, argv, "error_saver");
	ros::NodeHandle n;

	ofstream ResidualError;
	ResidualError.open ("/home/shreyanshdarshan/Localization/catkin_ws/ResidualError.txt", ios::out | ios::trunc);	
	ResidualError.close();
	// Subscribing the encoder data on /encoders
	// Also calls callback function which calculates velocity data from encoders
	ros::Subscriber odom_sub = n.subscribe<geometry_msgs::TwistStamped>("/ResidualError", 50, errorCallback);

	while (ros::ok())
		ros::spin();

	return 0;
}