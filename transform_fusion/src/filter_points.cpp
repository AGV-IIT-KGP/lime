#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

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

// Pointcloud for subscribing to input cloud
sensor_msgs::PointCloud2 pcin;

// Subscriber function to accept input cloud
void getCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
	pcin = *input;
	return;
}

// Removes ground points from input cloud
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
		// Remove all pts below z_thresh
        if (z_thresh - pt.z > 0)
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
                    }
                }
            }
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
	ros::init(argc, argv, "filter_points");

	ros::NodeHandle node;

	ros::Rate rate(1000.0);

	tf::TransformListener listener;
	ros::Subscriber sub = node.subscribe("/shifted_points", 1000, getCloud);
	ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1000);

	float z_thresh = stof(argv[1]);
	float leaf_size = stof(argv[2]);
    int K = stoi(argv[3]);
    int edge_thresh_dist = stoi(argv[4]);

	while (node.ok())
	{
        // Apply voxel filter on input cloud
        sensor_msgs::PointCloud2 voxelised_cloud = voxelFilter(pcin, leaf_size, leaf_size);
        // Apply edge filter on voxelised cloud
		sensor_msgs::PointCloud2 edge_cloud = extractEdges(voxelised_cloud, K, edge_thresh_dist);
        // Remove ground on the edge cloud
        sensor_msgs::PointCloud2 groundless_cloud = removeGround(edge_cloud, z_thresh);
        // Publish the final cloud
        cloud_pub.publish(groundless_cloud);
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}