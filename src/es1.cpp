/*
 * es1.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: Trevisiol Angelo
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer

int main (int argc, char** argv)
{
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../dataset/table_scene_lms400.pcd", *cloud_in) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	std::cout << "Loaded "
			<< cloud_in->width * cloud_in->height
			<< " data points from the pcd file. "
			<< std::endl;

	// Create a new cloud from the original one:
	// we want to put in cloud_out only points with x < 0
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < cloud_in->points.size(); i++)
	{
		pcl::PointXYZRGB p;
		p.x = cloud_in->points[i].x;
		p.y = cloud_in->points[i].y;
		p.z = cloud_in->points[i].z;
		if (cloud_in->points[i].x < 0) //point with x<0 colored in blue
		{
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud_out->points.push_back(p);
		}
	}

	// Visualization:
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  	int left_view, right_view;

  	// Draw output point cloud:
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left_view);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right_view);
	viewer.setBackgroundColor(0, 0, 0);
	//viewer.addCoordinateSystem(0.1);

	//add text to left and right view
	viewer.addText("Original cloud", 5, 5, "left_view_text", left_view);
	viewer.addText("Segmented cloud", 5, 5, "right_view_text", right_view);

  	viewer.addPointCloud(cloud_in, "original", left_view);
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_out);
  	viewer.addPointCloud(cloud_out,	rgb, "segmented", right_view);

	std::cout << "Visualization... "<< std::endl;

	viewer.spin ();

	return 0;
}
