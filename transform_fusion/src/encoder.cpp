//Author : Shreyansh Darshan ( https://github.com/ShreyanshDarshan )

#include "ros/time.h"
#include <boost/assign.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>

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

// Subscriber callback function
// Accepts geometry_msgs::Twist message containing encoder data
// Calculates the linear and angular velocity from encoder data.
void odomCallback(geometry_msgs::Twist encoder_msg) {
  // value.linear.x contains left wheel encoder data
  // value.linear.y contains right wheel odometry data
  vl = (encoder_msg.linear.x * 5) * fac * speed / 18;
  vr = (encoder_msg.linear.y * 5) * fac * speed / 18;

  // Calculating linear velocity and storing it in velocity_msg.twist.linear.x
  // for publishing
  velocity_msg.twist.linear.x = fabs(vl + vr) / 2;

  // Determining turn direction
  if (vr > vl) {
    // Calculating turning radius using left and right wheel speeds
    r = (dist * fabs(vl + vr)) / (2 * (vr - vl));

    // Calculating angular speed in radians per sec and storing it in
    // velocity_msg.twist.angular.z for publishing
    velocity_msg.twist.angular.z = fabs(vr) * speed / (r + dist / 2);
  } else {
    // Calculating turning radius using left and right wheel speeds
    r = (dist * (vl + vr)) / (2 * (vl - vr));

    // Calculating angular speed in radians per sec and storing it in
    // velocity_msg.twist.angular.z for publishing
    velocity_msg.twist.angular.z = -fabs(vl) * speed / (r + dist / 2);
  }

  // Assuming velocity perpendicular to vehicle's heading direction to be zero
  velocity_msg.twist.linear.y = 0.0;
  velocity_msg.twist.linear.z = 0.0;

  // Assuming vehicle's pitch and roll to be zero
  velocity_msg.twist.angular.x = 0.0;
  velocity_msg.twist.angular.y = 0.0;

  // Adding header data to the message
  velocity_msg.header.stamp = ros::Time::now();
  velocity_msg.header.frame_id = "odom";

  // Publishing the velocity data
  odom_pub.publish(velocity_msg);
}

int main(int argc, char **argv) {
  // Initialising the node
  ros::init(argc, argv, "encoder_data");
  ros::NodeHandle n;

  // Subscribing the encoder data on /encoders
  // Also calls callback function which calculates velocity data from encoders
  ros::Subscriber odom_sub =
      n.subscribe<geometry_msgs::Twist>("/encoders", 50, odomCallback);

  // Defining message type and topic for publishing
  odom_pub = n.advertise<geometry_msgs::TwistStamped>("/velocity_can", 50);

  while (ros::ok())
    ros::spin();

  return 0;
}