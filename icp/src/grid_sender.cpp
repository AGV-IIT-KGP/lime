#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//#include <eigen3/Eigen>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <bits/stdc++.h>

#define TILE_X 20
#define TILE_Y 20

using namespace std;
sensor_msgs::PointCloud2 pcin;
sensor_msgs::PointCloud2 pcout;
sensor_msgs::PointCloud2 pcout2;
void get_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
  // sensor_msgs::PointCloud2 pcin = *input;
  // sensor_msgs::convertPointCloud2ToPointCloud (temp, pcin);
  pcin = *input;
  return;
}
int main(int argc, char **argv)
{
  tf::StampedTransform transform1;
  tf::StampedTransform transform2;
  ros::init(argc, argv, "grid_map_publisher");

  ros::NodeHandle node;

  ros::Rate rate(1000.0);

  tf::TransformListener listener;
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/grid_map", 1000);

  float X_car, Y_car;
  int N1 = 0, N2 = 0, N1_prev = 0, N2_prev = 0;
  pcl::PointCloud<pcl::PointXYZ> map_cloud;

  map_cloud.clear();
  for (int i=-1; i<=1; i++)
  {
    for (int j=-1; j<=1; j++)
    {
  // for (int i = -15; i <= 20; i++)
  // {
  //   for (int j = -30; j <= 5; j++)
  //   {
      string path = "/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/grid_map/";
      string i_str = to_string(i + N1);
      string j_str = to_string(j + N2);
      path += i_str;
      path += "_";
      path += j_str;
      path += ".pcd";
      pcl::PointCloud<pcl::PointXYZ> tempCloud;
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, tempCloud) != -1)
      {
        map_cloud += tempCloud;
      }
    }
  }

  while (node.ok())
  {

    try
    {
      listener.lookupTransform("odom", "base_link", ros::Time(0), transform1);
      //cout<<transform.translation.x<<endl<<transform.translation.y<<endl<<endl;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Matrix3x3 m(transform1.getRotation());
    double r, p, y;
    m.getRPY(r, p, y);
    r = 0;
    p = 0;
    Eigen::Matrix4f Tm, Rm, CummTransform;
    CummTransform << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Rm << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Tm << 1, 0, 0, transform1.getOrigin().x(),
        0, 1, 0, transform1.getOrigin().y(),
        0, 0, 1, 0, //transform1.getOrigin().z(),
        0, 0, 0, 1;
    Rm(0, 0) = cos(y) * cos(p);
    Rm(0, 1) = (cos(y) * sin(p) * sin(r)) - (sin(y) * cos(r));
    Rm(0, 2) = (cos(y) * sin(p) * cos(r)) + (sin(y) * sin(r));
    Rm(1, 0) = sin(y) * cos(p);
    Rm(1, 1) = (sin(y) * sin(p) * sin(r)) + (cos(y) * cos(r));
    Rm(1, 2) = (sin(y) * sin(p) * cos(r)) - (cos(y) * sin(r));
    Rm(2, 0) = -sin(p);
    Rm(2, 1) = cos(p) * sin(r);
    Rm(2, 2) = cos(p) * cos(r);
    //std::cout<<Rm<<std::endl;
    // pcl_ros::transformPointCloud (Rm, pcin, pcout);
    // pcl_ros::transformPointCloud (Tm, pcout, pcout2);

   /* CummTransform = Tm * (Rm * CummTransform);

    try
    {
      listener.lookupTransform("base_link", "cumm", ros::Time(0), transform2);
      //cout<<transform.translation.x<<endl<<transform.translation.y<<endl<<endl;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    Eigen::Matrix4f Tm1, Rm1;
    m = tf::Matrix3x3(transform2.getRotation());
    //double r,p,y;
    m.getRPY(r, p, y);
    r = 0;
    p = 0;
    // Eigen::Matrix4f Tm, Rm;
    Rm1 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Tm1 << 1, 0, 0, transform2.getOrigin().x(),
        0, 1, 0, transform2.getOrigin().y(),
        0, 0, 1, 0, //transform2.getOrigin().z(),
        0, 0, 0, 1;
    Rm1(0, 0) = cos(y) * cos(p);
    Rm1(0, 1) = (cos(y) * sin(p) * sin(r)) - (sin(y) * cos(r));
    Rm1(0, 2) = (cos(y) * sin(p) * cos(r)) + (sin(y) * sin(r));
    Rm1(1, 0) = sin(y) * cos(p);
    Rm1(1, 1) = (sin(y) * sin(p) * sin(r)) + (cos(y) * cos(r));
    Rm1(1, 2) = (sin(y) * sin(p) * cos(r)) - (cos(y) * sin(r));
    Rm1(2, 0) = -sin(p);
    Rm1(2, 1) = cos(p) * sin(r);
    Rm1(2, 2) = cos(p) * cos(r);

    CummTransform = Tm1 * (Rm1 * CummTransform);*/

    X_car = Tm(0, 3);
    Y_car = Tm(1, 3);

    // cout<<X_car<<" "<<Y_car<<endl<<Tm<<endl;

    int N1 = X_car / TILE_X, N2 = Y_car / TILE_Y;

    if (N1_prev != N1 || N2_prev != N2)
    {
      map_cloud.clear();
      cout << N1 << " " << N2 << endl;
      for (int i = -1; i <= 1; i++)
      {
        for (int j = -1; j <= 1; j++)
        {
          string path = "/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/grid_map/";
          string i_str = to_string(i + N1);
          string j_str = to_string(j + N2);
          path += i_str;
          path += "_";
          path += j_str;
          path += ".pcd";
          pcl::PointCloud<pcl::PointXYZ> tempCloud;
          if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, tempCloud) != -1)
          {
            map_cloud += tempCloud;
          }
        }
      }
    }
    // loadNewMap();

    N1_prev = N1;
    N2_prev = N2;

    //cout<<transform1.getOrigin().x()<<endl;
    sensor_msgs::PointCloud2 object_msg;
    pcl::toROSMsg(map_cloud, object_msg);
    object_msg.header.frame_id = "odom";
    cloud_pub.publish(object_msg);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}