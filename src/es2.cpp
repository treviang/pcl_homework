/*
 * es2.cpp
 *
 *	Create a point cloud obtained applying four different values of
 *	the leaf size of the voxel grid filter to different parts of the
 *	input point cloud
 *
 *  Created on: Dec 29, 2014
 *      Author: Trevisiol Angelo
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>					// for voxel grid filtering
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int
main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_bn (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/table_scene_lms400.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from the pcd file. "
			<< std::endl;
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
				<< " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create four new clouds from the original one:
	// we want to put in cloud1 points with x < 0, y < 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	// we want to put in cloud2 points with x < 0, y > 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	// we want to put in cloud3 points with x > 0, y < 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
	// we want to put in cloud4 points with x > 0, y > 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);

	//fulfill the 4 new small point clouds
	for (int i = 0; i < cloud->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		if ((cloud->points[i].x < 0) && (cloud->points[i].y < 0))
		{
			cloud1->points.push_back(p);
		}
		else if ((cloud->points[i].x < 0) && (cloud->points[i].y > 0))
		{
			cloud2->points.push_back(p);
		}
		else if ((cloud->points[i].x > 0) && (cloud->points[i].y < 0))
		{
			cloud3->points.push_back(p);
		}
		else
		{
			cloud4->points.push_back(p);
		}
	}

	std::cerr << "CLoud 1: " << cloud1->points.size() <<" points." << std::endl;
	std::cerr << "CLoud 2: " << cloud2->points.size() <<" points." << std::endl;
	std::cerr << "CLoud 3: " << cloud3->points.size() <<" points." << std::endl;
	std::cerr << "CLoud 4: " << cloud4->points.size() <<" points." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	//leaf size 0.005
	sor.setInputCloud (cloud1);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter (*cloud1);
	//leaf size 0.01
	sor.setInputCloud (cloud2);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud2);
	//leaf size 0.05
	sor.setInputCloud (cloud3);
	sor.setLeafSize (0.05f, 0.05f, 0.05f);
	sor.filter (*cloud3);
	//leaf size 0.1
	sor.setInputCloud (cloud4);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (*cloud4);

	//reassembles the point cloud
	*cloud_filtered_bn = *cloud1;
	*cloud_filtered_bn += *cloud2;
	*cloud_filtered_bn += *cloud3;
	*cloud_filtered_bn += *cloud4;

	//change colors
	for (int i = 0; i < cloud_filtered_bn->points.size(); i++)
	{
		pcl::PointXYZRGB p;
		p.x = cloud_filtered_bn->points[i].x;
		p.y = cloud_filtered_bn->points[i].y;
		p.z = cloud_filtered_bn->points[i].z;
		if (cloud_filtered_bn->points[i].x < 0 && cloud_filtered_bn->points[i].y < 0)
		{
			//points with x < 0, y < 0 color red
			p.r = 255;
			p.g = 0;
			p.b = 0;
			cloud_filtered->points.push_back(p);
		}
		else if (cloud_filtered_bn->points[i].x < 0 && cloud_filtered_bn->points[i].y > 0)
		{
			//points with x < 0, y > 0 color green
			p.r = 0;
			p.g = 255;
			p.b = 0;
			cloud_filtered->points.push_back(p);
		}
		else if (cloud_filtered_bn->points[i].x > 0 && cloud_filtered_bn->points[i].y < 0)
		{
			//points with x > 0, y < 0 color blue
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud_filtered->points.push_back(p);
		}
		else
		{
			//points with x > 0, y > 0 with color white
			p.r = 255;
			p.g = 255;
			p.b = 255;
			cloud_filtered->points.push_back(p);
		}
	}

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// Draw output point cloud:
	viewer.setBackgroundColor (0.3, 0.3, 0.3);
	//viewer.addCoordinateSystem (0.1, "cloud");
	viewer.addText ("Cloud Filtered", 10, 10);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "cloud");

	// Loop for visualization (so that the visualizers are continuously updated):
	std::cout << "Visualization... "<< std::endl;

	viewer.spin ();

	return 0;
}
