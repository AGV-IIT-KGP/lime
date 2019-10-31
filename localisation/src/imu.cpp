//Node converts NED output of IMU into ENU and adds covariance with timestamps Outputs on imu
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <boost/assign.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#define PI 3.14159265359
ros::Publisher imu_pub, imu_pub2;
sensor_msgs::Imu imu;
using namespace std_msgs;

void imuCallback(const sensor_msgs::Imu msg)
{
    imu=msg;
    imu.orientation_covariance=boost::assign::list_of(0.001)(0.0)(0.0)
    	                                                (0.0)(0.001)(0.0)
    	                                                (0.0)(0.0)(0.0001);

    imu.angular_velocity_covariance=boost::assign::list_of(0.001)(0.0)(0.0)
                                                  (0.0)(0.001)(0.0)
                                                  (0.0)(0.0)(0.0001);
    imu.linear_acceleration_covariance=boost::assign::list_of(10000)(0.0)(0.0)
                                                      (0.0)(0.001)(0.0)
                                                      (0.0)(0.0)(0.0001);
    imu.header.frame_id="imu";

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    //convert to ENU
    yaw=yaw+(PI/2.0);
    pitch=0.0;
    roll=0.0;

    tf::Quaternion q;
    q.setRPY(tfScalar(roll), tfScalar(pitch), tfScalar(yaw));
    Float64 yawf;
    yawf.data=yaw*(180.0/3.1415926);
    
    
    imu_pub2.publish(yawf);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    imu.orientation = odom_quat;
    imu.header.stamp=ros::Time::now();
    imu_pub.publish(imu);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");

    ros::NodeHandle n;
    imu_pub2=n.advertise<Float64>("/imu/orientation",1);
    ros::Subscriber imu_sub=n.subscribe<sensor_msgs::Imu>("/vn_ins/imu",50,imuCallback);

    imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 50);
    ros::spin();
    return 0;
}

