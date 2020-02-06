#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;

ofstream file_1("transforms.txt");

float dist (pcl::PointXYZ a, pcl::PointXYZ b)
{
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}

int i=0;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cld(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cld(new pcl::PointCloud<pcl::PointXYZ>);

int main (int argc, char** argv)
{
	ros::init(argc, argv, "map_publisher");
    // ros::package::getPath('PKG_NAME')
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/media/shreyanshdarshan/New Volume/vision/PCL/PCD_VIEW/build/civil.pcd", *map_cld);
	ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1000);
    sensor_msgs::PointCloud2 map_msg;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (map_cld);
    pcl::PointXYZ searchPoint;

    cout<<endl<<"started"<<endl;

    cout<< "Enter threshold distance" << endl;
    float thresholdDistance = 1;
    cin >> thresholdDistance;

    // K nearest neighbor search
    int K = 20;
    cout<<"Enter K: "<<endl;
    cin >> K;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

    int sz = (*map_cld).size();

    for (int iter=0; iter<(*map_cld).size(); iter++)
    {
        searchPoint = map_cld->points[iter];
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            pcl::PointXYZ Centroid;
            Centroid.x = Centroid.y = Centroid.z = 0;
            int num = 0;
            float ResDist = 10000;
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                if (dist(searchPoint, map_cld->points[ pointIdxNKNSearch[i] ]) > 0.001)
                {
                    Centroid.x += map_cld->points[ pointIdxNKNSearch[i] ].x;
                    Centroid.y += map_cld->points[ pointIdxNKNSearch[i] ].y;
                    Centroid.z += map_cld->points[ pointIdxNKNSearch[i] ].z;
                    num++;
                    float resCheck = dist (searchPoint, map_cld->points[ pointIdxNKNSearch[i] ]);
                    if (resCheck < ResDist)
                    {
                        ResDist = resCheck;
                        // cout<<dist(searchPoint, map_cld->points[ pointIdxNKNSearch[i] ])<<endl;
                    }
                }
                // std::cout << "    "  <<   map_cld->points[ pointIdxNKNSearch[i] ].x 
                //         << " " << map_cld->points[ pointIdxNKNSearch[i] ].y 
                //         << " " << map_cld->points[ pointIdxNKNSearch[i] ].z 
                //         << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
            }
            // cout<<                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   endl;
            if (num>0)
            {
                Centroid.x /= num;
                Centroid.y /= num;
                Centroid.z /= num;
            }

            if (dist (Centroid, searchPoint) > thresholdDistance*ResDist)
            {
                (*edge_cld).push_back(searchPoint);
            }
            // cout << iter <<" "<< sz << endl;
        }
    }
    cout << endl << "ended" << endl;

    pcl::toROSMsg(*edge_cld.get(), map_msg );
    map_msg.header.frame_id = "odom";
    while (ros::ok())
    {
        map_pub.publish(map_msg);
        //cout<<object_msg;
	    // ros::spinOnce();
        break;
    }
	return 0;
}