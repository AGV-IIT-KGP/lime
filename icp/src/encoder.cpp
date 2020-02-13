#include <ros/ros.h>
#include <boost/assign.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include "ros/time.h"
#define distance 1.45

geometry_msgs::TwistStamped msg;
float vl,vr,r;
ros::Publisher odom_pub;
void odomCallback(geometry_msgs::Twist value)
{

    vl=(value.linear.x*5)/18;
    vr=(value.linear.y*5)/18;
    msg.twist.linear.x=fabs(vl+vr)/2;
    if(vr>vl)
      {
        r=(distance*fabs(vl+vr))/(2*(vr-vl));
        msg.twist.angular.z= fabs(vr)/(r+distance/2); //omega as radian per sec
      } 
    else
      {
        r=(distance*(vl+vr))/(2*(vl-vr));
        msg.twist.angular.z= -fabs(vl)/(r+distance/2); //omega as radian per sec
      } 

    msg.twist.linear.y=0.0;
    msg.twist.linear.z=0.0;
    msg.twist.angular.x=0.0;
    msg.twist.angular.y=0.0;
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id="odom";
    odom_pub.publish(msg);
}
int main(int argc, char** argv){
 
  ros::init(argc, argv, "encoder_data");
  ros::NodeHandle n;
  //odometry - covariance + tf
  ros::Subscriber odom_sub=n.subscribe<geometry_msgs::Twist>("/encoders",50,odomCallback);
  odom_pub = n.advertise<geometry_msgs::TwistStamped>("/velocity_can", 50);
  while(ros::ok()) ros::spin();
  return 0;
}