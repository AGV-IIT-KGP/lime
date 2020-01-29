#include <iostream>
#include "ros/ros.h"
 #include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bits/stdc++.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "opencv2/opencv.hpp" 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define ORIG_X 200
#define ORIG_Y 550
#define SCALE 1.7

#define TILE_X 20
#define TILE_Y 20
#define M_S_X1 -15
#define M_S_X2 20
#define M_S_Y1 -30
#define M_S_Y2 5

// #define TILE_X 50
// #define TILE_Y 50
// #define M_S_X1 -3
// #define M_S_X2 3
// #define M_S_Y1 -6
// #define M_S_Y2 0

using namespace std;
using namespace pcl;
using namespace cv;
ofstream file_1("transforms.txt");
// if (pcl::io::loadPCDFile<pcl::PointXYZ> ("map.pcd", *cloud_out) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read the map file i.e map.pcd \n");
//     return (-1);
//   }
int i=0;
Eigen::Matrix4f cumm_transform;
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_lidar(new pcl::PointCloud<pcl::PointXYZ>);

void plot (Mat &img, float x, float y)
{
    y = y*SCALE+ORIG_Y;
    x = x*SCALE+ORIG_X;
    if (y<img.rows && y>=0 && x<img.cols && x>=0)
    {
        if (img.at<uchar>(y, x)<255)
        img.at<uchar>(y, x) += 1;
    }
}

void rect (Mat &img, float x1, float x2, float y1, float y2)
{
    y1 = y1*SCALE+ORIG_Y;
    x1 = x1*SCALE+ORIG_X;
    y2 = y2*SCALE+ORIG_Y;
    x2 = x2*SCALE+ORIG_X;
    rectangle(img, Point(x1, y1), Point(x2, y2), Scalar(30));
}

void pc2_to_pcl_plus_icp(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *current_lidar);
}
int main (int argc, char** argv)
{
	ros::init(argc, argv, "ICP_on_map");
	ros::NodeHandle n;
    cout<<endl<<"adada"<<endl;
    // pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/high res map/kitti final map.pcd", *cloud_out);//
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/map.pcd", *cloud_out);
    ros::Subscriber sub = n.subscribe("/map_cloud", 1000, pc2_to_pcl_plus_icp);
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1000);
    //ros::Publisher tf_pub = n.advertise<tf::Transform>("/corr_matrix", 1000);
    

    cumm_transform << 1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1;
   // cumm_transform=transform_1*cumm_transform;

    tf::Vector3 origin;
    origin.setValue(static_cast<double>(cumm_transform(0,3)),static_cast<double>(cumm_transform(1,3)), static_cast<double>(cumm_transform(2,3)));

    cout << origin << endl;
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(cumm_transform(0,0)), static_cast<double>(cumm_transform(0,1)), static_cast<double>(cumm_transform(0,2)), 
            static_cast<double>(cumm_transform(1,0)), static_cast<double>(cumm_transform(1,1)), static_cast<double>(cumm_transform(1,2)), 
            static_cast<double>(cumm_transform(2,0)), static_cast<double>(cumm_transform(2,1)), static_cast<double>(cumm_transform(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "cumm"));   

    //OPENCV PART
    Mat Mimg = Mat(700, 700, CV_8UC1, Scalar(0));
    Mat Fimg = Mat(700, 700, CV_8UC1, Scalar(0));
    // circle(img, Point(350, 350), 50, Scalar (100), 4);

    cout<<endl<<(*cloud_out).size()<<endl;

    for (int i=0; i<(*cloud_out).size(); i++)
    {
        float x = (*cloud_out)[i].x, y = (*cloud_out)[i].y; 
        plot(Mimg, x, y);
    }

    for (int i=M_S_X1; i<=M_S_X2; i++)
    {
        for (int j=M_S_Y1; j<=M_S_Y2; j++)
        {
            rect (Mimg, i*TILE_X, (i+1)*TILE_X, j*TILE_Y, (j+1)*TILE_Y);
        }
    }

    pcl::PointCloud<pcl::PointXYZ> Maps[M_S_X2-M_S_X1+1][M_S_Y2-M_S_Y1+1];

    for (int i=0; i<(*cloud_out).size(); i++)
    {
        if ( (*cloud_out)[i].x >= M_S_X1*TILE_X && (*cloud_out)[i].x < (M_S_X2+1)*TILE_X && (*cloud_out)[i].y >= M_S_Y1*TILE_Y && (*cloud_out)[i].y < (M_S_Y2+1)*TILE_Y )
        {
            Maps[int((*cloud_out)[i].x/(TILE_X))-M_S_X1][int((*cloud_out)[i].y/(TILE_Y))-M_S_Y1].push_back((*cloud_out)[i]);
        }
    }

    for (int i=M_S_X1; i<=M_S_X2; i++)
    {
        for (int j=M_S_Y1; j<=M_S_Y2; j++)
        {
            string path = "/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/grid_map/";
            string i_str = to_string(i);
            string j_str = to_string(j);
            path += i_str;
            path += "_";
            path += j_str;
            path += ".pcd";
            cout<<path<<" ";
            if (Maps[i-M_S_X1][j-M_S_Y1].size())
            {
                cout<<pcl::io::savePCDFile (path, Maps[i-M_S_X1][j-M_S_Y1])<<endl;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    waitKey(0);
    for (int i=M_S_X1; i<=M_S_X2; i++)
    {
        for (int j=M_S_Y1; j<=M_S_Y2; j++)
        {
            string path = "/home/shreyanshdarshan/Localization/catkin_ws/src/premapped_localization/icp/src/grid_map/";
            string i_str = to_string(i);
            string j_str = to_string(j);
            path += i_str;
            path += "_";
            path += j_str;
            path += ".pcd";
            pcl::PointCloud<pcl::PointXYZ> tempCloud;
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, tempCloud) != -1)
            {
                map_cloud += tempCloud;
            }
        }
    }

    while (ros::ok())
    {
        Fimg = Mat::zeros(Fimg.rows, Fimg.cols, CV_8UC1);
        for (int i=0; i<(map_cloud).size(); i++)
        {
            float x = (map_cloud)[i].x, y = (map_cloud)[i].y; 
            plot(Fimg, x, y);
        }   

        for (int i=M_S_X1; i<=M_S_X2; i++)
        {
            for (int j=M_S_Y1; j<=M_S_Y2; j++)
            {
                rect (Fimg, i*TILE_X, (i+1)*TILE_X, j*TILE_Y, (j+1)*TILE_Y);
            }
        }

        // sensor_msgs::PointCloud2 object_msg;
        // pcl::toROSMsg(*transformed_cloud.get(),object_msg );
        // object_msg.header.frame_id = "odom";
        // cloud_pub.publish(object_msg);

        
        imshow ("MIMGWIN", Mimg);
        imshow ("FIMGWIN", Fimg);
        waitKey(1);

	    ros::spinOnce();
    }
	return 0;
}