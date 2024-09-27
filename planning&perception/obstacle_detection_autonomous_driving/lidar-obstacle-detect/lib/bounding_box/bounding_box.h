#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <cmath>
#include <limits>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>

class BoundingBox
{
public:
    BoundingBox();
    BoundingBox(ros::NodeHandle pnh);
    ~BoundingBox();

    void SetCloud(std_msgs::Header header, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, bool in_estimate_pose);
    void getBoundingBox(std_msgs::Header header,
                        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector,
                        autoware_msgs::CloudClusterArray &inOutClusters);
    void ToROSMessage(std_msgs::Header header, autoware_msgs::CloudCluster &outClusterMessage);
    jsk_recognition_msgs::BoundingBox GetBoundingBox();
    geometry_msgs::PolygonStamped GetPolygon();

    bool in_estimate_pose_;

    bool validCluster_;
    pcl::PointXYZ centroid_;
    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ average_point_;
    jsk_recognition_msgs::BoundingBox bounding_box_;
    geometry_msgs::PolygonStamped polygon_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud_;

    Eigen::Matrix3f eigen_vectors_;
    Eigen::Vector3f eigen_values_;
    double orientation_angle_;

    double cluster_merge_threshold_;
};

typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

#endif
