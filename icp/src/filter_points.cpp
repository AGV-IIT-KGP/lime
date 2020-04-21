#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//#include <eigen3/Eigen>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
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

using namespace std;

sensor_msgs::PointCloud2 pcin;

void get_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	// sensor_msgs::PointCloud2 pcin = *input;
	// sensor_msgs::convertPointCloud2ToPointCloud (temp, pcin);
	pcin = *input;
	return;
}

sensor_msgs::PointCloud2 removeGround(sensor_msgs::PointCloud2 input_cloud, int z_thresh)
{
	sensor_msgs::PointCloud2 output_cloud;
	pcl::PCLPointCloud2 groundless;
	pcl_conversions::toPCL(input_cloud, groundless);
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundless_P(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(groundless, *groundless_P);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	for (int i = 0; i < (*groundless_P).size(); i++)
	{
		pcl::PointXYZ pt(groundless_P->points[i].x, groundless_P->points[i].y, groundless_P->points[i].z);
		if (z_thresh - pt.z > 0) // e.g. remove all pts below zAvg
		{
			inliers->indices.push_back(i);
		}
	}
	if ((*groundless_P).size() > 0)
	{
		extract.setInputCloud(groundless_P);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*groundless_P);
		pcl::toROSMsg(*groundless_P.get(), output_cloud);
	}

	output_cloud.header.frame_id = "odom";
	return output_cloud;
}

sensor_msgs::PointCloud2 voxelFilter(sensor_msgs::PointCloud2 input_cloud, float leafxy, float leafz)
{
	sensor_msgs::PointCloud2 output_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_P(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(input_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *output_cloud_P);
	
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leafxy, leafxy, leafz);
    grid.setInputCloud (output_cloud_P);
    grid.filter (*output_cloud_P);

	pcl::toROSMsg(*output_cloud_P.get(), output_cloud);
	return output_cloud;
}

float dist (pcl::PointXYZ a, pcl::PointXYZ b)
{
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}

sensor_msgs::PointCloud2 extractEdges(sensor_msgs::PointCloud2 input_cloud, int K, float thresholdDistance)
{
	sensor_msgs::PointCloud2 output_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cld(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 pcl_pc2;

	pcl_conversions::toPCL(input_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *unfiltered);
	(*edge_cld).clear();
    pcl::fromPCLPointCloud2(pcl_pc2, *unfiltered);

    // pcl::VoxelGrid<pcl::PointXYZ> grid;
    // grid.setLeafSize (leaf, leaf, leaf);
    // grid.setInputCloud (unfiltered);
    // grid.filter (*unfiltered);
    
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

	pcl::toROSMsg(*edge_cld.get(), output_cloud);
	output_cloud.header.frame_id = "odom";
	return output_cloud;	
}


int main(int argc, char **argv)
{
	tf::StampedTransform transform1;
	ros::init(argc, argv, "Filter_Cloud");

	ros::NodeHandle node;

	ros::Rate rate(1000.0);

	tf::TransformListener listener;
	ros::Subscriber sub = node.subscribe("/shifted_points", 1000, get_cloud);
	ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1000);

	float zAvg = -1.6;

	while (node.ok())
	{
		// cloud_pub.publish(removeGround(extractEdges(voxelFilter(pcin, 0.3, 0.3), 30, 1), zAvg));
		cloud_pub.publish(removeGround(pcin, zAvg));
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}