/*
 * @Author: xiaohu
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: xiaohu
 * @Last Modified time: 2022-04-02 01:12:59
 */

#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <future>

#include <ros/ros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

class EuclideanCluster
{
public:
  EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~EuclideanCluster(){};
  void cluster_vector(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, std::vector<pcl::PointIndices> &indices);
  void segmentByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr,
                         std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector);

  void clusterIndicesMultiThread(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_max_cluster_distance,
                                 std::promise<std::vector<pcl::PointIndices>> &promiseObj);

private:
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  std::vector<double> clustering_distances_;
  std::vector<double> clustering_ranges_;
  bool use_multiple_thres_;

  std::mutex mutex_; //先定义互斥锁
};

#endif
