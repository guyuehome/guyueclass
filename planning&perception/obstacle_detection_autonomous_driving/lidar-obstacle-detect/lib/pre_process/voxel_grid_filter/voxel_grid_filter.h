/*
 * @Author: xiaohu
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: xiaohu
 * @Last Modified time: 2022-04-02 01:12:59
 */

#ifndef DOWNSAMPLER_H
#define DOWNSAMPLER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <unistd.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>

class VoxelGridFilter
{
public:
  VoxelGridFilter();
  VoxelGridFilter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);

  ~VoxelGridFilter();

  void downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud);
  //下采样：体素滤波参数
  double leafSize;
  bool isDownsample;
};

#endif // DOWNSAMPLER_H
