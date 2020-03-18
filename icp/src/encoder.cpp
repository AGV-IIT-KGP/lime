#include <ros/ros.h>
#include <boost/assign.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include "ros/time.h"
#include<iostream>
// #define dist 1.45
#define fac 1.5
#define speed 1

using namespace std;

geometry_msgs::TwistStamped msg;
float vl,vr,r;
float dist =1.45;
ros::Publisher odom_pub;
void odomCallback(geometry_msgs::Twist value)
{

    vl=(value.linear.x*5)*fac*speed/18;
    vr=(value.linear.y*5)*fac*speed/18;
    msg.twist.linear.x=fabs(vl+vr)/2;
    if(vr>vl)
      {
        r=(dist*fabs(vl+vr))/(2*(vr-vl));
        msg.twist.angular.z= fabs(vr)*speed/(r+dist/2); //omega as radian per sec
      } 
    else
      {
        r=(dist*(vl+vr))/(2*(vl-vr));
        msg.twist.angular.z= -fabs(vl)*speed/(r+dist/2); //omega as radian per sec
      } 

    msg.twist.linear.y=0.0;
    msg.twist.linear.z=0.0;
    // msg.twist.linear.x=0.0;
    // msg.twist.angular.z=0.0;
    msg.twist.angular.x=0.0;
    msg.twist.angular.y=0.0;
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id="odom";
    odom_pub.publish(msg);
}
int main(int argc, char** argv){
//   cout<<"dist: "<<endl;
//   cin>>dist;
  ros::init(argc, argv, "encoder_data");
  ros::NodeHandle n;
  //odometry - covariance + tf
  ros::Subscriber odom_sub=n.subscribe<geometry_msgs::Twist>("/encoders",50,odomCallback);
  odom_pub = n.advertise<geometry_msgs::TwistStamped>("/velocity_can", 50);
  while(ros::ok()) ros::spin();
  return 0;
}