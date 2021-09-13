#pragma once

// !PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

//!MSgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

//!opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointXYZ    PointT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointT> Cloud;
typedef pcl::PointCloud<PointCT> ColorCloud;

#include "pcl_util.h"
//! Message transform
void ROS2PCL(const sensor_msgs::PointCloudConstPtr packet,
             Cloud::Ptr cloud_out);

//! get rectangle area of point clouds
void subrectangle(Cloud::Ptr cloud_in,
                  Cloud::Ptr cloud_out,
                  double cut_max_x,
                  double cut_min_x,
                  double cut_max_y,
                  double cut_min_y,
                  double cut_max_z,
                  double cut_min_z);

//! Plane Extraction
bool GetPlane(Cloud::Ptr cloud_in,
              ColorCloud::Ptr cloudPlane,
              PointCT &centroid_point,
              pcl::ModelCoefficients::Ptr coeffs,
              pcl::PointIndices::Ptr indices, double threshold);

//!draw point cloud with image color
void drawPointcloudColor(Cloud::Ptr cloud_in,
                         ColorCloud::Ptr cloud_out,
                         const cv::Mat& img,
                         const cv::Mat& R,
                         const cv::Mat& T,
                         const cv::Mat& cameraMat,
                         const cv::Mat& distCoeff,
                         const cv::Size& imageSize);
