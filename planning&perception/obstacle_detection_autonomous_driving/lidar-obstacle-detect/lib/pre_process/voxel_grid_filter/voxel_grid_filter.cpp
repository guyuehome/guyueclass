/*
 * @Author: z
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: z
 * @Last Modified time: 2022-04-02 01:12:59
 */
#include "voxel_grid_filter.h"

VoxelGridFilter::VoxelGridFilter() {}

/**
 * @brief VoxelGridFilter 构造函数
 *
 * 初始化 VoxelGridFilter 类对象，使用给定的 ros::NodeHandle 对象进行节点句柄的初始化。
 *
 * @param node_handle ROS 节点句柄
 * @param private_node_handle ROS 私有节点句柄
 */
VoxelGridFilter::VoxelGridFilter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
  /* Initialize tuning parameter */
  private_node_handle.param("is_downsample", isDownsample, true);
  private_node_handle.param("leaf_size", leafSize, 0.1);
}

VoxelGridFilter::~VoxelGridFilter() {}

/**
 * @brief 下采样函数
 *
 * 使用体素网格滤波器对输入的点云进行下采样，并将结果存储在输出点云中。
 *
 * @param in_cloud 输入的点云指针
 * @param out_cloud 输出的点云指针
 */
void VoxelGridFilter::downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud)
{
  // // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (leafSize >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(in_cloud);
    //参数为float
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*out_cloud);
  }
  else
  {
    out_cloud = in_cloud;
  }
}
