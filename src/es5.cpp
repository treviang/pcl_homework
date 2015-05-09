/*
 * es5.cpp
 *
 *  Created on: Jan 21, 2015
 *
 *  Register together a dataset of point clouds
 *
 *      Author: Trevisiol Angelo
 */

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// visualizer
pcl::visualization::PCLVisualizer *p;
//left and right viewports
int vp_1, vp_2;

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h (cloud_target);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_h (cloud_source);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  p-> spinOnce();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_color (cloud_target);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_color (cloud_source);

  p->addPointCloud (cloud_target, tgt_color, "target", vp_2);
  p->addPointCloud (cloud_source, src_color, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief  Filter robot points from the whole point cloud
  * \param original point cloud from the original dataset
  *
  */
int filterRobot(PointCloud::Ptr &original){

  PointCloud::Ptr cloud_filtered (new PointCloud);
  //range of PassThrought filter
  float min = 0.0f;
  float max = 2.0f;

  //excludes background from the Point Cloud by a PassThrough filter
  pcl::PassThrough<pcl::PointXYZRGB> pth;
  pth.setInputCloud(original);
  pth.setFilterFieldName("z");
  pth.setFilterLimits(min, max);
  pth.filter(*cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  double seg_threshold = 0.02;  //segmentation threshold of 2cm
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  PointCloud::Ptr cloud_plane (new PointCloud ());

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (seg_threshold);
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
    return -1;

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  original=cloud_filtered;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
  void loadData (int argc, char **argv, std::vector<PointCloud::Ptr> &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PointCloud::Ptr p (new PointCloud());
      pcl::io::loadPCDFile (argv[i], *p);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*p,*p, indices);

      models.push_back (p);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  */
  Eigen::Matrix4f pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr traslate_src (new PointCloud());
  PointCloud::Ptr traslate_tgt (new PointCloud());

  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  if (downsample)
  {
    grid.setLeafSize (0.01, 0.01, 0.01);
    grid.setInputCloud (cloud_src);
    grid.filter (*traslate_src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*traslate_tgt);
  }
  else
  {
    traslate_src = cloud_src;
    traslate_tgt = cloud_tgt;
  }

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB,pcl::PointXYZRGB> reg; //ICP Non Linear object
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 30cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.3);
  //set RANSAC outlier removal distance threshold to 5cm
  reg.setRANSACOutlierRejectionThreshold(0.05);
  reg.setRANSACIterations(30);

  reg.setInputSource (traslate_src);
  reg.setInputTarget (traslate_tgt);

  reg.setMaximumIterations (3);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), targetToSource;
  PointCloud::Ptr reg_result = traslate_src;

  int iterations=30;
  for (int i = 0; i < iterations; ++i)
  {
    // save cloud for visualization purpose
    traslate_src = reg_result;

    // Estimate
    reg.setInputSource (traslate_src);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    // visualize current state
    showCloudsRight(traslate_tgt, traslate_src);
  }

  //refining registration with Iterative Closest Point Linear
  pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icpLin;

  icpLin.setInputSource (reg_result);
  icpLin.setInputTarget (traslate_tgt);
  icpLin.setMaximumIterations (2000);
  icpLin.align(*reg_result);
  showCloudsRight(traslate_tgt, reg_result);
  Ti = icpLin.getFinalTransformation() * Ti;

  std::cout << (icpLin.hasConverged() ? "Alignment succeeded!" : "Alignment failed.") << std::endl;
  std::cout << "Alignment score: " << icpLin.getFitnessScore() << std::endl;
  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();
  return targetToSource;
 }

/* ---[ */
int main (int argc, char** argv)
{
  std::cout <<"Loading point clouds...\n"<<std::flush;
  // Load data into point cloud vector
  std::vector<PointCloud::Ptr> data;
  clock_t t_start,t_load,t_seg,t_align_start,t_align_end;  //timers

  t_start = clock();
  loadData (argc, argv, data);
  t_load = clock();

  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  std::cout<< "Loaded " <<(int)data.size() <<" in " <<(t_load-t_start)*1000/CLOCKS_PER_SEC <<" ms" <<std::endl;

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  std::cout <<"Performing robot segmentation of point clouds... "<<std::flush;

  //segment the robot
  t_seg = clock();
  for(size_t i = 0; i < data.size (); ++i){
    if(filterRobot(data[i])){
      PCL_ERROR ("Could not segment robot in this Point Cloud");
    }
  }
  std::cout <<"Segmentation completed in " <<(t_seg-t_load)*1000/CLOCKS_PER_SEC <<" ms" <<std::endl;

  PointCloud::Ptr source (new PointCloud), target;
  Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity ();
  std::vector<Eigen::Matrix4f> globalTransforms(data.size());
  //inizialize globalTransform matrix
  globalTransforms[0] = Eigen::Matrix4f::Identity ();
  for (size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1]; //load source
    target = data[i]; //load target

    // Add visualization data
    showCloudsLeft(source, target);

    PointCloud::Ptr temp (new PointCloud);
    //perform aligning
    std::cout <<"Aligning " <<i+1 <<" with " <<i <<"...   " <<std::flush;
    t_align_start = clock();
    pairTransform = pairAlign (source, target, true);
    t_align_end = clock();
    std::cout <<"Alignment completed in " <<(t_align_end-t_align_start)*1000/CLOCKS_PER_SEC <<" ms" <<std::endl;
    //update the global transform matrix
    globalTransforms[i] = Eigen::Matrix4f::Identity ();
    globalTransforms[i] = globalTransforms[i-1]* pairTransform;
  }

  // Initialize new viewer for final result:
  pcl::visualization::PCLVisualizer registration_viewer("Nao registration complete");   // viewer initialization

  PointCloud::Ptr result (new PointCloud);
  for (size_t i = 0; i < data.size (); ++i)
  {
    source = data[i];
    //transorm original point clouds according to global transform matrix
    pcl::transformPointCloud (*source, *result, globalTransforms[i]);
    //visualize new point cloud
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> result_color_handler (result);
    registration_viewer.addPointCloud (result, result_color_handler, "result"+i);

  }

  registration_viewer.spin();
  return 0;
}
/* ]--- */
