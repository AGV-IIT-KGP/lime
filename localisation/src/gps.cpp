//Node adds coveriance and timestamp to gps data and publishes on fix_c
#include <ros/ros.h>
#include <boost/assign.hpp>
#include <sensor_msgs/NavSatFix.h>
#include "ros/time.h"
#include<iostream>
using namespace std;

ros::Publisher gps_pub;
void gps(sensor_msgs::NavSatFix msg)
{
    sensor_msgs::NavSatFix vn_ins;
    vn_ins=msg;
    vn_ins.header.stamp =ros::Time::now();
    vn_ins.status.service=1;
    vn_ins.position_covariance_type=1;
    vn_ins.position_covariance=boost::assign::list_of(100.0)(0.0)(0.0)
                                              (0.0)(100.0)(0.0)
                                              (0.0)(0.0)(100.0);
    vn_ins.header.frame_id="gps";                       
    gps_pub.publish(vn_ins);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps");

    ros::NodeHandle n;
    ros::Subscriber gps_sub=n.subscribe<sensor_msgs::NavSatFix>("/vn_ins/fix",50,gps);
    gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix_c", 50);
    ros::spin();
    return 0;
}