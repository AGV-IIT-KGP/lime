//Node adds yaw_dot of the IMU to the encoder data to fuse into odom_imu_combined
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <geometry_msgs/TwistStamped.h>

#define PI 3.1415926

using namespace ros;
ros::Publisher odom_pub;
double vl, vr, v;
geometry_msgs::Quaternion quat;
nav_msgs::Odometry odom;


void odomCallback(const geometry_msgs::TwistStamped msg)
{
    std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n..........\n\n\n\n\n\n\n\n\n";
    vr=msg.twist.linear.y/**(5.0/18)*/;//fix for kmph to mps for mahindra car
    vl=msg.twist.linear.x/**(5.0/18)*/;
    v=(vl+vr)/2;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0;

    //publish the message
    odom_pub.publish(odom);
}

void imuCallback(sensor_msgs::Imu imu_msg)
{
    quat= imu_msg.orientation;
    odom.twist.twist.angular=imu_msg.angular_velocity;
}

int main(int argc, char** argv)
{
    std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n..........\n\n\n\n\n\n\n\n\n";
    ros::init(argc, argv, "odom_imu");
    ros::NodeHandle n;

    odom_pub = n.advertise<nav_msgs::Odometry>("/odom_imu_combined", 1);
    ros::Subscriber odom_sub=n.subscribe<geometry_msgs::TwistStamped>("/velocity_can",1, &odomCallback);   // Roboteq publishes on result_vel
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/vn_ins/imu", 1, &imuCallback);

    while(ros::ok()) ros::spin();
}
