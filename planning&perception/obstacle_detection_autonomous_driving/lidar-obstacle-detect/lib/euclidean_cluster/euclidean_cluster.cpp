/*
 * @Author: xiaohu
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: xiaohu
 * @Last Modified time: 2022-04-02 01:12:59
 */

#include "euclidean_cluster.h"

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    pnh.param("cluster_tolerance", cluster_tolerance_, 0.35);
    pnh.param("min_cluster_size", min_cluster_size_, 5);
    pnh.param("max_cluster_size", max_cluster_size_, 20000);
    pnh.param("use_multiple_thres", use_multiple_thres_, false);

    clustering_distances_ = {0.5, 1.1, 1.6, 2.1, 2.6};
    clustering_ranges_ = {15, 30, 45, 60};
}
void EuclideanCluster::cluster_vector(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, std::vector<pcl::PointIndices> &indices)
{
    //设置查找方式－kdtree
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_); 
    ec.setMinClusterSize(min_cluster_size_);   
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(indices);
}

/**
 * @brief 根据距离进行欧几里得聚类分割
 *
 * 根据给定的点云数据，根据距离进行欧几里得聚类分割，将分割后的点云数据存储在指定的输出点云指针和点云向量中。
 *
 * @param in 输入的点云数据指针
 * @param out_cloud_ptr 输出的点云数据指针
 * @param points_vector 存储分割后点云数据的向量
 */
void EuclideanCluster::segmentByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector)
{
    std::vector<pcl::PointIndices> cluster_indices;
    if (!use_multiple_thres_)
    {
        cluster_vector(in, cluster_indices);
        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*in, it->indices, *temp_cloud_ptr);
            *out_cloud_ptr += *temp_cloud_ptr;
            points_vector.push_back(temp_cloud_ptr);
        }
    }
    else
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_segments_array(5);
        for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            cloud_segments_array[i] = tmp_cloud;
        }

        for (unsigned int i = 0; i < in->points.size(); i++)
        {
            pcl::PointXYZI current_point;
            current_point.x = in->points[i].x;
            current_point.y = in->points[i].y;
            current_point.z = in->points[i].z;
            current_point.intensity = in->points[i].intensity;

            float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

            if (origin_distance < clustering_ranges_[0])
            {
                cloud_segments_array[0]->points.push_back(current_point);
            }
            else if (origin_distance < clustering_ranges_[1])
            {
                cloud_segments_array[1]->points.push_back(current_point);
            }
            else if (origin_distance < clustering_ranges_[2])
            {
                cloud_segments_array[2]->points.push_back(current_point);
            }
            else if (origin_distance < clustering_ranges_[3])
            {
                cloud_segments_array[3]->points.push_back(current_point);
            }
            else
            {
                cloud_segments_array[4]->points.push_back(current_point);
            }
        }

        std::vector<std::thread> thread_vec(cloud_segments_array.size());
        for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
        {
            // 这种获取多线程返回值写法，运行速度慢，大家有兴趣自行更改，我懒改了，这是粗版demo
            std::promise<std::vector<pcl::PointIndices>> promiseObj;
            std::shared_future<std::vector<pcl::PointIndices>> futureObj = promiseObj.get_future();
            thread_vec[i] = std::thread(&EuclideanCluster::clusterIndicesMultiThread, this, cloud_segments_array[i], std::ref(clustering_distances_[i]), std::ref(promiseObj));
            cluster_indices = futureObj.get();
            for (int j = 0; j < cluster_indices.size(); j++)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::copyPointCloud(*cloud_segments_array[i], cluster_indices[j], *temp_cloud_ptr);
                *out_cloud_ptr += *temp_cloud_ptr;
                points_vector.push_back(temp_cloud_ptr);
            }
        }

        for (int i = 0; i < thread_vec.size(); i++)
        {
            thread_vec[i].join();
        }
    }
}

/**
 * @brief 使用多线程进行欧几里得聚类
 *
 * 根据给定的点云数据，使用多线程进行欧几里得聚类，并将聚类结果通过 std::promise 返回。
 *
 * @param in_cloud_ptr 输入的点云数据指针
 * @param in_max_cluster_distance 最大的聚类距离
 * @param promiseObj 用于返回聚类结果的 std::promise 对象
 */
void EuclideanCluster::clusterIndicesMultiThread(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_max_cluster_distance,
                                                 std::promise<std::vector<pcl::PointIndices>> &promiseObj)
{
    // make it flat
    // for (size_t i = 0; i < cloud_2d->points.size(); i++)
    // {
    //     cloud_2d->points[i].z = 0;
    // }
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_); 
    ec.setMinClusterSize(min_cluster_size_);   
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud_ptr);
    ec.extract(indices);

    promiseObj.set_value(indices);
}


