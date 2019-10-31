#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <bits/stdc++.h>
#include <ros/ros.h> 


using namespace std;
using namespace sensor_msgs;
int skip = 0;
int count1, count2, count3;
ofstream testgps1, testgps2, testgps3;
void gpsCb1(const sensor_msgs::NavSatFixConstPtr& gps )
{
    count1++;
    if(count1>skip)
{    double latitude=gps->latitude;
    double longitude=gps->longitude;
    testgps1 << std::fixed << std::setprecision(9) << latitude << "," << longitude << "," <<"Kharagpur,#FFFF00"<< endl;
    count1=0;}
}
void gpsCb2(const sensor_msgs::NavSatFixConstPtr& gps )
{
    count2++;
    if(count2>skip)
{    double latitude=gps->latitude;
    double longitude=gps->longitude;
    testgps2 << std::fixed << std::setprecision(9) << latitude << "," << longitude << "," <<"Kharagpur,#FFFF00"<< endl;
    count2=0;}
}
void gpsCb3(const sensor_msgs::NavSatFixConstPtr& gps )
{
    count3++;
    if(count3>skip)
{    double latitude=gps->latitude;
    double longitude=gps->longitude;
    testgps3 << std::fixed << std::setprecision(9) << latitude << "," << longitude << "," <<"Kharagpur,#FFFF00"<< endl;
    count2=0;}
}
int main(int argc, char** argv)
{
    std::cout << std::setprecision(9);
    ros::init(argc, argv, "mobilgps");   
    ros::NodeHandle nh;
    ros::Subscriber gps1 = nh.subscribe("/vn_ins/fix", 1, &gpsCb1);
    ros::Subscriber gps2 = nh.subscribe("/android/fix", 1, &gpsCb2);
    ros::Subscriber gps3 = nh.subscribe("/gps/filtered", 1, &gpsCb3);
    testgps1.open("/home/vib2810/igvc_ws/testgps1.txt"); 
    testgps2.open("/home/vib2810/igvc_ws/testgps2.txt"); 
    testgps3.open("/home/vib2810/igvc_ws/testgps3.txt"); 
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}