/*
 * es6.cpp
 *
 *  Automatic ground plane estimator
 *
 *  Created on: Jan 16, 2015
 *      Author: Trevisiol Angelo
 */

/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a PCD file.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm    <path_to_svm_file>" << std::endl;
  cout << "   --data     <path_to_dataset_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "   --sample  <sampling_factor (default = 1)>" << std::endl;
  cout << "*******************************************************" << std::endl;
  return 0;
}

int main (int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

  /// Dataset Parameters:
  std::string filename = "../dataset/people/five_people";
  std::string svm_filename = "../dataset/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;
  float sampling_factor = 1;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
  clock_t t_start,t_ground,t_people;  //timer

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--data", filename);
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument (argc, argv, "--min_h", min_height);
  pcl::console::parse_argument (argc, argv, "--max_h", max_height);
  pcl::console::parse_argument (argc, argv, "--sample", sampling_factor);

  // Read Kinect data:
  PointCloudT::Ptr cloud = PointCloudT::Ptr (new PointCloudT);
  PointCloudT::Ptr cloud_filtered = PointCloudT::Ptr (new PointCloudT);

  if (pcl::io::loadPCDFile(filename + ".pcd", *cloud) < 0)
  {
    cerr << "Failed to read test file" <<filename <<".pcd." << endl;
    return (-1);
  }

  // Create classifier for people detection:
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  //people_detector.setHeightLimits(min_height, max_height);         // set person classifier
  people_detector.setSamplingFactor(sampling_factor);              // set a downsampling factor to the point cloud (for increasing speed)
  //people_detector.setSensorPortraitOrientation(true);              // set sensor orientation to vertical

  /// PCL viewer ///
  pcl::visualization::PCLVisualizer viewer("Automatic ground plane estimator");
  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  //Automatic Ground plane estimation:
  t_start=clock();

  //voxel grid filtering to speedup the ground plane estimation
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  //segmentation of the filtered pointcloud
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional parameters
  seg.setOptimizeCoefficients (true);
  // Mandatory parameters
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (6000);
  seg.setDistanceThreshold (0.05);
  seg.setAxis(Eigen::Vector3f(0, 1, 0));
  seg.setEpsAngle(0.6);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0) //no plane detected
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  Eigen::VectorXf ground_coeffs; //copy model coefficients into VectorXf object
  ground_coeffs.resize(4);
  for(int i=0; i < coefficients->values.size() ; i++){
    ground_coeffs[i]=coefficients->values[i];
  }
  std::cout << "Automatic ground plane estimated: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Extract inliers
  pcl::ExtractIndices<PointT> extract;
  pcl::PointCloud<PointT>::Ptr ground_plane (new pcl::PointCloud<PointT>);
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);    // set filter method to return "outliers" instead of "inliers"
  extract.filter (*ground_plane);
  std::cerr << "PointCloud representing the planar component: " << ground_plane->width * ground_plane->height << " data points." << std::endl;

  // Draw auto-estimated ground plane in original pointcloud
  viewer.addPointCloud(ground_plane,
      pcl::visualization::PointCloudColorHandlerCustom<PointT>(ground_plane, 255,0,0),
      "ground");

  t_ground=clock();
  std::cout<< "Ground plane estimated in " <<(t_ground-t_start)*1000/CLOCKS_PER_SEC <<" ms" <<std::endl;
  // Perform people detection on the new cloud:
  std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
  people_detector.setInputCloud(cloud);
  people_detector.setGround(ground_coeffs);                    // set floor coefficients
  people_detector.compute(clusters);                           // perform people detection
  ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer people_viewer("People detection");          // viewer initialization
  people_viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Draw cloud and people bounding boxes in the viewer:
  people_viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");

  unsigned int k = 0;
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
    {
      // draw theoretical person bounding box in the PCL viewer:
      it->drawTBoundingBox(people_viewer, k);
      k++;
    }
  }
  std::cout << k << " people found" << std::endl;
  t_people=clock();
  std::cout<< "People founded in " <<(t_people-t_ground)*1000/CLOCKS_PER_SEC <<" ms" <<std::endl;
  people_viewer.spin();

  return 0;
}
