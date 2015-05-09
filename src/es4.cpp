/*
 * es4.cpp
 *
 *  Created on: Dec 30, 2014
 *
 *  compute FPFH features only at keypoint's locations
 *
 *      Author: Trevisiol Angelo
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 		    // for reading the point cloud
#include <pcl/visualization/pcl_visualizer.h>		// for PCL visualizer
#include <pcl/visualization/histogram_visualizer.h>	// for histogram visualization
#include <pcl/features/normal_3d.h>	    // for computing normals
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud


struct callbackArgs{
	// structure used to pass arguments to the callback function
	pcl::visualization::PCLHistogramVisualizer *histViewer;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;
};

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
	// callback function used to visualize feature histograms
	struct callbackArgs* data = (struct callbackArgs *)args;
    if (event.getPointIndex () == -1)
      return;
    std::stringstream windowName;
    windowName << "FPFH for point " << event.getPointIndex();
    data->histViewer->addFeatureHistogram (*(data->fpfhs), "fpfh", event.getPointIndex(), windowName.str(), 640, 200);
}

int main (int argc, char** argv){
	// Variables declaration:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

	//parameters of keypoints detection
	float min_scale = 0.01;
	int nr_octaves = 3;
 	int nr_scales_per_octave = 2;
 	int min_contrast = 0;

 	//parameters of feature estimation
 	float feature_radius=0.05;

	// Load point cloud from .pcd file:
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../dataset/minimouse1.pcd", *cloud) == -1) //* load the file
	//if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../dataset/minimouse1_segmented.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return (-1);
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from the pcd file. "
			<< std::endl;

	// Remove NaN from the point cloud:
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	// Create the sift keypoint class, and pass the input dataset to it
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
	sift_detect.setInputCloud(cloud);

	// Create an empty FLANN-based KdTree representation,to perform neighborhood searches.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	sift_detect.setSearchMethod(tree);

	// set detection parameters
	sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast (min_contrast);

	// Output datasets
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_out (new pcl::PointCloud<pcl::PointWithScale>);

	// Compute the keypoints
	std::cout << "Computing keypoints...please wait..." <<std::flush;
	sift_detect.compute (*keypoints_out);
	std::cout << "done." << std::endl;

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

	// Create the normal estimation class, and pass the input dataset to it
	//  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the normals:
	std::cout << "Computing normals...please wait..." <<std::flush;
	ne.setNumberOfThreads(8); 	// set number of threads when using OpenMP
	ne.compute (*cloud_normals);
	std::cout << "done." << std::endl;

	// Convert the keypoints cloud from PointWithScale to PointXYZRGB
	// so that it will be compatible with our original point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*keypoints_out, *keypoints_xyzrgb);

	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;

	fpfh.setSearchSurface(cloud);
	fpfh.setInputCloud (keypoints_xyzrgb);
	fpfh.setInputNormals (cloud_normals);
	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (feature_radius);

	// Compute the features
	std::cout << "Computing FPFH features...please wait..." <<std::flush;
	fpfh.setNumberOfThreads(8); 	// set number of threads when using OpenMP
	fpfh.compute (*fpfhs);
	std::cout << "done." << std::endl;

	// Visualize FPFH:
	int normalsVisualizationStep = 100; // to visualize a normal every normalsVisualizationStep
	float normalsScale = 0.02;			// normals dimension

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.initCameraParameters ();
	//viewer.addCoordinateSystem(0.1);
	viewer.setBackgroundColor(0, 0, 0.5);

	int left_viewport, right_viewport;
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left_viewport);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right_viewport);
	viewer.addText("Original point cloud", 10, 10, "left_viewport_label", left_viewport);
	viewer.addText("Keypoints", 10, 10, "right_viewport_label", right_viewport);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler (keypoints_xyzrgb, 255, 255, 255);
	//left side
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud , rgb , "input_cloud_left" , left_viewport );
	//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals , normalsVisualizationStep , normalsScale, "normals", left_viewport);
	//right side
  viewer.addPointCloud(keypoints_xyzrgb , keypoints_color_handler , "keypoints_cloud", right_viewport);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_cloud");

  pcl::visualization::PCLHistogramVisualizer histViewer;
	// Create structure containing the parameters for the callback function
	struct callbackArgs histCallbackArgs;
	histCallbackArgs.histViewer = &histViewer;
	histCallbackArgs.fpfhs = fpfhs;

	// Add point picking callback to viewer (for visualizing feature histograms):
	viewer.registerPointPickingCallback (pp_callback, (void*)&histCallbackArgs);

    std::cout << "Visualization... "<< std::endl;
    while (!viewer.wasStopped ())
	{
			viewer.spin();
  		histViewer.spin();

	}
  	return 0;
}
