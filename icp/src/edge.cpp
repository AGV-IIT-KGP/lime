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
#include <pcl/filters/voxel_grid.h>

#define leaf 0.4

using namespace std;

ofstream file_1("transforms.txt");

float dist (pcl::PointXYZ a, pcl::PointXYZ b)
{
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}

int i=0;
pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr in_cld(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2 pcl_pc2;
pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cld(new pcl::PointCloud<pcl::PointXYZ>);

void pc2_to_pcl(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
  pcl_conversions::toPCL(*input, pcl_pc2);
}

void edge_extract(int K, int thresholdDistance)
{
    (*edge_cld).clear();    
    pcl::fromPCLPointCloud2(pcl_pc2, *unfiltered);
    
    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (unfiltered);
    grid.filter (*unfiltered);
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (unfiltered);
    pcl::PointXYZ searchPoint;

    cout<<endl<<"started"<<endl;

    // K nearest neighbor search
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

    int sz = (*unfiltered).size();

    int edge_pts=0, non_edge_pts = 0;

    for (int iter=0; iter<(*unfiltered).size(); iter++)
    {
        searchPoint = unfiltered->points[iter];
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            pcl::PointXYZ Centroid;
            Centroid.x = Centroid.y = Centroid.z = 0;
            int num = 0;
            float ResDist = 10000;
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                if (dist(searchPoint, unfiltered->points[ pointIdxNKNSearch[i] ]) > 0.001)
                {
                    Centroid.x += unfiltered->points[ pointIdxNKNSearch[i] ].x;
                    Centroid.y += unfiltered->points[ pointIdxNKNSearch[i] ].y;
                    Centroid.z += unfiltered->points[ pointIdxNKNSearch[i] ].z;
                    num++;
                    float resCheck = dist (searchPoint, unfiltered->points[ pointIdxNKNSearch[i] ]);
                    if (resCheck < ResDist)
                    {
                        ResDist = resCheck;
                        // cout<<dist(searchPoint, unfiltered->points[ pointIdxNKNSearch[i] ])<<endl;
                    }
                }
                // std::cout << "    "  <<   unfiltered->points[ pointIdxNKNSearch[i] ].x 
                //         << " " << unfiltered->points[ pointIdxNKNSearch[i] ].y 
                //         << " " << unfiltered->points[ pointIdxNKNSearch[i] ].z 
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
                edge_pts ++;
            }
            else
            {
                non_edge_pts++;
            }
            
            // cout << iter <<" "<< sz << endl;
        }
    }
    cout << endl << "ended\nedge pts = " << edge_pts << "\nnon edge pts = " << non_edge_pts<<endl;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "edge_pub");
    // ros::package::getPath('PKG_NAME')
    //pcl::io::loadPCDFile<pcl::PointXYZ> ("/media/shreyanshdarshan/New Volume/vision/PCL/XYZ2PCD/build/civil_rotated.pcd", *unfiltered);
	
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, pc2_to_pcl);
    ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/edge_cloud", 1000);
    sensor_msgs::PointCloud2 edge_msg;

    cout<< "Enter threshold distance" << endl;
    float thresholdDistance = 1;
    cin >> thresholdDistance;
    int K = 20;
    cout<<"Enter K: "<<endl;
    cin >> K;

    while (ros::ok())
    {
        edge_extract(K, thresholdDistance);
        pcl::toROSMsg(*edge_cld.get(), edge_msg );
        edge_msg.header.frame_id = "velodyne";
        map_pub.publish(edge_msg);
        //cout<<object_msg;
	    ros::spinOnce();
        // break;
    }
	return 0;
}