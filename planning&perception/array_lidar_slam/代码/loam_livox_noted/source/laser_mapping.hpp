// This is the Lidar Odometry And Mapping (LOAM) for solid-state lidar (for example: livox lidar),
// which suffer form motion blur due the continously scan pattern and low range of fov.

// Developer: Jiarong Lin  ziv.lin.ljr@gmail.com

//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LASER_MAPPING_HPP
#define LASER_MAPPING_HPP

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <future>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <vector>

#include "cell_map_keyframe.hpp"
#include "ceres_icp.hpp"
#include "ceres_pose_graph_3d.hpp"
#include "point_cloud_registration.hpp"
#include "scene_alignment.hpp"
#include "tools/common.h"
#include "tools/pcl_tools.hpp"
#include "tools/tools_logger.hpp"
#include "tools/tools_timer.hpp"

#define PUB_SURROUND_PTS 1
#define PCD_SAVE_RAW 1
#define PUB_DEBUG_INFO 0
#define IF_PUBLISH_SURFACE_AND_CORNER_PTS 0

// int g_if_undistore = 0; //0: 分段处理； 1： 线性插值； 作者默认为0
int g_if_undistore = 1; //而且 yaml中common/if_motion_deblur必须为 1
//也就是说g_if_undistore和if_motion_deblur必须同时为1，才是线性插值去模糊

int if_motion_deblur = 0;
double history_add_t_step = 0.00;
double history_add_angle_step = 0.00;

using namespace PCL_TOOLS;
using namespace Common_tools;

struct Data_pair {
    sensor_msgs::PointCloud2ConstPtr m_pc_corner;
    sensor_msgs::PointCloud2ConstPtr m_pc_full;
    sensor_msgs::PointCloud2ConstPtr m_pc_plane;
    bool m_has_pc_corner = 0;
    bool m_has_pc_full = 0;
    bool m_has_pc_plane = 0;

    void add_pc_corner(sensor_msgs::PointCloud2ConstPtr ros_pc) {
        m_pc_corner = ros_pc;
        m_has_pc_corner = true;
    }

    void add_pc_plane(sensor_msgs::PointCloud2ConstPtr ros_pc) {
        m_pc_plane = ros_pc;
        m_has_pc_plane = true;
    }

    void add_pc_full(sensor_msgs::PointCloud2ConstPtr ros_pc) {
        m_pc_full = ros_pc;
        m_has_pc_full = true;
    }

    bool is_completed() {
        return (m_has_pc_corner & m_has_pc_full & m_has_pc_plane);
    }
};

class Point_cloud_registration;

class Laser_mapping {
    public:
    int m_current_frame_index = 0;
    int m_para_min_match_blur = 0.0;
    int m_para_max_match_blur = 0.3;
    int m_max_buffer_size = 50000000;

    int m_mapping_init_accumulate_frames = 100;
    int m_kmean_filter_count = 3;
    int m_kmean_filter_threshold = 2.0;

    double m_time_pc_corner_past = 0;
    double m_time_pc_surface_past = 0;
    double m_time_pc_full = 0;
    double m_time_odom = 0;               //not used
    double m_last_time_stamp = 0;         //本帧点中最大的时间戳
    double m_minimum_pt_time_stamp = 0;   //上一帧点中最大的时间戳
    double m_maximum_pt_time_stamp = 1.0; //本帧点中最大的时间戳
    float m_last_max_blur = 0.0;

    int m_odom_mode;
    int m_matching_mode = 0;
    int m_if_input_downsample_mode = 1;
    int m_maximum_parallel_thread;
    int m_maximum_mapping_buff_thread = 1; // Maximum number of thead for matching buffer update
    int m_maximum_history_size = 100;
    int m_para_threshold_cell_revisit = 0;
    
    float m_para_max_angular_rate = 200.0 / 50.0;//yaml: 4.0
    float m_para_max_speed = 100.0 / 50.0;//yaml: 2.0
    float m_max_final_cost = 100.0; //yaml: 2.0

    int m_para_icp_max_iterations = 20;
    int m_para_cere_max_iterations = 100;
    int m_para_optimization_maximum_residual_block = 1e5;
    double m_minimum_icp_R_diff = 0.01;
    double m_minimum_icp_T_diff = 0.01;

    string m_pcd_save_dir_name, m_log_save_dir_name, m_loop_save_dir_name;

    std::list<pcl::PointCloud<PointType>> m_laser_cloud_corner_history;  //算上当前帧在内，最多保留400帧 histor frames, 最top的最oldest
    std::list<pcl::PointCloud<PointType>> m_laser_cloud_surface_history; //算上当前帧在内，最多保留400帧 history frames
    std::list<pcl::PointCloud<PointType>> m_laser_cloud_full_history;    //算上当前帧在内，最多保留100帧 history frames（因为会不时地被reset）

    std::list<double> m_his_reg_error;                                    //每一帧点云与local map匹配计算出位姿后的cost，the smaller the more certain about calculated pose.
    Eigen::Quaterniond m_last_his_add_q = Eigen::Quaterniond::Identity(); //TODO author not initialized, we add
    Eigen::Vector3d m_last_his_add_t = Eigen::Vector3d::Zero();

    //
    std::map<int, float> m_map_life_time_corner;
    std::map<int, float> m_map_life_time_surface;

    // ouput: all visualble cube points
    pcl::PointCloud<PointType>::Ptr m_laser_cloud_surround;

    // surround points in map to build tree
    int m_if_mapping_updated_corner = true;
    int m_if_mapping_updated_surface = true;

    pcl::PointCloud<PointType>::Ptr m_laser_cloud_corner_from_map; //not used
    pcl::PointCloud<PointType>::Ptr m_laser_cloud_surf_from_map;   //not used

    pcl::PointCloud<PointType>::Ptr m_laser_cloud_corner_from_map_last; //local map corners
    pcl::PointCloud<PointType>::Ptr m_laser_cloud_surf_from_map_last;   //local map plannes

    //input & output: points in one frame. local --> global
    pcl::PointCloud<PointType>::Ptr m_laser_cloud_full_res; //当前帧全部点云

    // input: from odom
    pcl::PointCloud<PointType>::Ptr m_laser_cloud_corner_last; //当前帧corners点云
    pcl::PointCloud<PointType>::Ptr m_laser_cloud_surf_last;   //当前帧plannes点云

    //kd-tree
    pcl::KdTreeFLANN<PointType> m_kdtree_corner_from_map; //not used
    pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map;   //not used

    pcl::KdTreeFLANN<PointType> m_kdtree_corner_from_map_last; //local_map_kdtree_corners
    pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map_last;   //local_map_kdtree_plannes

    int m_laser_cloud_valid_Idx[1024];
    int m_laser_cloud_surround_Idx[1024];

    const Eigen::Quaterniond m_q_I = Eigen::Quaterniond(1, 0, 0, 0);

    double m_para_buffer_RT[7] = {0, 0, 0, 1, 0, 0, 0};
    double m_para_buffer_RT_last[7] = {0, 0, 0, 1, 0, 0, 0};

    Eigen::Map<Eigen::Quaterniond> m_q_w_curr = Eigen::Map<Eigen::Quaterniond>(m_para_buffer_RT); //当前帧在map下位姿
    Eigen::Map<Eigen::Vector3d> m_t_w_curr = Eigen::Map<Eigen::Vector3d>(m_para_buffer_RT + 4);

    Eigen::Map<Eigen::Quaterniond> m_q_w_last = Eigen::Map<Eigen::Quaterniond>(m_para_buffer_RT_last); //上一帧在map下位姿
    Eigen::Map<Eigen::Vector3d> m_t_w_last = Eigen::Map<Eigen::Vector3d>(m_para_buffer_RT_last + 4);

    std::map<double, Data_pair*> m_map_data_pair; //<时间戳, laser_data>
    std::queue<Data_pair*> m_queue_avail_data;    //时间戳小的排在前, 最多保持 maximum_mapping_buffer帧laser

    std::queue<nav_msgs::Odometry::ConstPtr> m_odom_que;
    std::mutex m_mutex_buf;

    float m_line_resolution = 0;
    float m_plane_resolution = 0;
    pcl::VoxelGrid<PointType> m_down_sample_filter_corner;
    pcl::VoxelGrid<PointType> m_down_sample_filter_surface;
    pcl::StatisticalOutlierRemoval<PointType> m_filter_k_means;

    std::vector<int> m_point_search_Idx;
    std::vector<float> m_point_search_sq_dis;

    nav_msgs::Path m_laser_after_mapped_path, m_laser_after_loopclosure_path;

    int m_if_save_to_pcd_files = 1;
    PCL_point_cloud_to_pcd m_pcl_tools_aftmap;
    PCL_point_cloud_to_pcd m_pcl_tools_raw;

    Common_tools::File_logger m_logger_common;
    Common_tools::File_logger m_logger_loop_closure;
    Common_tools::File_logger m_logger_pcd;
    Common_tools::File_logger m_logger_timer;
    Common_tools::File_logger m_logger_matching_buff;
    Scene_alignment<float> m_sceene_align; //not useful
    Common_tools::Timer m_timer;

    ros::Publisher m_pub_laser_cloud_surround, m_pub_laser_cloud_map, m_pub_laser_cloud_full_res, m_pub_odom_aft_mapped, m_pub_odom_aft_mapped_hight_frec;
    ros::Publisher m_pub_laser_aft_mapped_path, m_pub_laser_aft_loopclosure_path;
    ros::NodeHandle m_ros_node_handle;
    ros::Subscriber m_sub_laser_cloud_corner_last, m_sub_laser_cloud_surf_last, m_sub_laser_odom, m_sub_laser_cloud_full_res;

    ceres::Solver::Summary m_final_opt_summary;
    //std::list<std::thread* > m_thread_pool;
    std::list<std::future<int>*> m_thread_pool;
    std::list<std::future<void>*> m_thread_match_buff_refresh;

    double m_maximum_in_fov_angle;
    double m_maximum_pointcloud_delay_time;
    double m_maximum_search_range_corner;
    double m_maximum_search_range_surface;
    double m_surround_pointcloud_resolution;
    double m_lastest_pc_reg_time = -3e8;
    double m_lastest_pc_matching_refresh_time = -3e8;
    double m_lastest_pc_income_time = -3e8;

    std::mutex m_mutex_mapping;
    std::mutex m_mutex_querypointcloud;
    std::mutex m_mutex_buff_for_matching_corner;
    std::mutex m_mutex_buff_for_matching_surface;
    std::mutex m_mutex_thread_pool;
    std::mutex m_mutex_ros_pub;
    std::mutex m_mutex_dump_full_history;
    std::mutex m_mutex_keyframe;

    float m_pt_cell_resolution = 1.0;
    Points_cloud_map<float> m_pt_cell_map_full;    //全部帧corners+plannes在map下坐标
    Points_cloud_map<float> m_pt_cell_map_corners; //全部帧corners在map下坐标
    Points_cloud_map<float> m_pt_cell_map_planes;  //全部帧plannes在map下坐标

    int m_down_sample_replace = 1;
    ros::Publisher m_pub_last_corner_pts, m_pub_last_surface_pts;
    ros::Publisher m_pub_match_corner_pts, m_pub_match_surface_pts, m_pub_debug_pts, m_pub_pc_aft_loop;
    std::future<void> *m_mapping_refresh_service_corner, *m_mapping_refresh_service_surface, *m_mapping_refresh_service; // Thread for mapping update
    std::future<void> *m_service_pub_surround_pts, *m_service_loop_detection;                                            // Thread for loop detection and publish surrounding pts

    Common_tools::Timer timer_all;
    std::mutex timer_log_mutex;

    int m_if_maps_incre_update_mean_and_cov;
    int m_loop_closure_if_enable;
    int m_loop_closure_if_dump_keyframe_data;
    int m_loop_closure_minimum_keyframe_differen;
    int m_para_scans_of_each_keyframe = 0;
    int m_para_scans_between_two_keyframe = 0;
    int m_para_scene_alignments_maximum_residual_block;
    int m_loop_closure_map_alignment_maximum_icp_iteration;
    int m_loop_closure_map_alignment_if_dump_matching_result;
    int m_loop_closure_maximum_keyframe_in_wating_list;

    float m_loop_closure_minimum_similarity_linear;
    float m_loop_closure_minimum_similarity_planar;
    float m_loop_closure_map_alignment_resolution;
    float m_loop_closure_map_alignment_inlier_threshold;

    // ANCHOR keyframe define
    //std::shared_ptr<Maps_keyframe<float>>            m_current_keyframe;
    std::list<std::shared_ptr<Maps_keyframe<float>>> m_keyframe_of_updating_list;     //还没有累积够300帧的keyframes
    std::list<std::shared_ptr<Maps_keyframe<float>>> m_keyframe_need_precession_list; //保存累积了300帧的所有keyframes

    ADD_SCREEN_PRINTF_OUT_METHOD;

    //判断点是否在视野内
    int if_pt_in_fov(const Eigen::Matrix<double, 3, 1>& pt) {

        auto pt_affine = m_q_w_curr.inverse() * (pt - m_t_w_curr); //转换到当前帧下

        if (pt_affine(0) < 0)
            return 0;

        float angle = Eigen_math::vector_angle(pt_affine, Eigen::Matrix<double, 3, 1>(1, 0, 0), 1);

        if (angle * 57.3 < m_maximum_in_fov_angle)
            return 1;
        else
            return 0;
    }

    // void service_update_buff_for_matching_surface() {
    //     pcl::VoxelGrid<PointType> down_sample_filter_surface = m_down_sample_filter_surface;

    //     while (1) {
    //         if (m_if_mapping_updated_surface == false) {
    //             std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    //             continue;
    //         } else {
    //             pcl::PointCloud<PointType>::Ptr laser_cloud_surf_from_map(new pcl::PointCloud<PointType>());
    //             if (m_matching_mode) {
    //                 pcl::VoxelGrid<PointType> down_sample_filter_surface = m_down_sample_filter_surface;
    //                 std::vector<Points_cloud_map<float>::Mapping_cell_ptr> plane_cell_vec = m_pt_cell_map_planes.find_cells_in_radius(m_t_w_curr, m_maximum_search_range_surface);
    //                 int surface_cell_numbers_full = plane_cell_vec.size();
    //                 int surface_cell_numbers_in_fov = 0;
    //                 pcl::PointCloud<PointType> pc_temp;
    //                 for (size_t i = 0; i < plane_cell_vec.size(); i++) {
    //                     //*laser_cloud_surf_from_map +=  PCL_TOOLS::eigen_pt_to_pcl_pointcloud<PointType>( plane_cell_vec[i]->m_points_vec );
    //                     int if_in_fov = if_pt_in_fov(plane_cell_vec[i]->m_center.cast<double>());
    //                     if (if_in_fov == 0) {
    //                         continue;
    //                     }
    //                     surface_cell_numbers_in_fov++;

    //                     down_sample_filter_surface.setInputCloud(plane_cell_vec[i]->m_pcl_pc_vec.makeShared());
    //                     down_sample_filter_surface.filter(pc_temp);
    //                     if (m_down_sample_replace) {//1
    //                         plane_cell_vec[i]->set_pointcloud(pc_temp);
    //                     }
    //                     *laser_cloud_surf_from_map += pc_temp;
    //                 }
    //                 screen_printf("==== Ratio of surface in fovs %.2f ====\r\n",
    //                               (float)surface_cell_numbers_in_fov / surface_cell_numbers_full);
    //             } else {
    //                 for (auto it = m_laser_cloud_surface_history.begin(); it != m_laser_cloud_surface_history.end(); it++) {
    //                     *laser_cloud_surf_from_map += (*it);
    //                 }
    //             }

    //             down_sample_filter_surface.setInputCloud(laser_cloud_surf_from_map);
    //             down_sample_filter_surface.filter(*laser_cloud_surf_from_map);
    //             if (laser_cloud_surf_from_map->points.size()) {
    //                 m_kdtree_surf_from_map.setInputCloud(laser_cloud_surf_from_map);
    //             }

    //             m_if_mapping_updated_surface = false;
    //             m_mutex_buff_for_matching_surface.lock();
    //             *m_laser_cloud_surf_from_map_last = *laser_cloud_surf_from_map;
    //             m_kdtree_surf_from_map_last = m_kdtree_surf_from_map;
    //             m_mutex_buff_for_matching_surface.unlock();
    //         }
    //     }
    // }

    // void service_update_buff_for_matching_corner() {
    //     pcl::VoxelGrid<PointType> down_sample_filter_corner = m_down_sample_filter_corner;

    //     while (1) {
    //         if (m_if_mapping_updated_corner == false) {
    //             std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    //             continue;
    //         } else {
    //             pcl::PointCloud<PointType>::Ptr laser_cloud_corner_from_map(new pcl::PointCloud<PointType>());
    //             if (m_matching_mode) {
    //                 pcl::VoxelGrid<PointType> down_sample_filter_corner = m_down_sample_filter_corner;
    //                 std::vector<Points_cloud_map<float>::Mapping_cell_ptr> corner_cell_vec = m_pt_cell_map_corners.find_cells_in_radius(m_t_w_curr, m_maximum_search_range_corner);
    //                 int corner_cell_numbers_full = corner_cell_vec.size();
    //                 int corner_cell_numbers_in_fov = 0;

    //                 pcl::PointCloud<PointType> pc_temp;

    //                 for (size_t i = 0; i < corner_cell_vec.size(); i++) {
    //                     //*laser_cloud_corner_from_map +=  PCL_TOOLS::eigen_pt_to_pcl_pointcloud<PointType>( corner_cell_vec[i]->m_points_vec );
    //                     int if_in_fov = if_pt_in_fov(corner_cell_vec[i]->m_center.cast<double>());
    //                     if (if_in_fov == 0) {
    //                         continue;
    //                     }
    //                     corner_cell_numbers_in_fov++;
    //                     down_sample_filter_corner.setInputCloud(corner_cell_vec[i]->m_pcl_pc_vec.makeShared());
    //                     down_sample_filter_corner.filter(pc_temp);
    //                     if (m_down_sample_replace) {//1
    //                         corner_cell_vec[i]->set_pointcloud(pc_temp);
    //                     }
    //                     *laser_cloud_corner_from_map += pc_temp;
    //                 }

    //                 screen_printf("==== Ratio of corner in fovs %.2f ====\r\n", (float)corner_cell_numbers_in_fov / corner_cell_numbers_full);
    //             } else {
    //                 for (auto it = m_laser_cloud_corner_history.begin(); it != m_laser_cloud_corner_history.end(); it++) {
    //                     *laser_cloud_corner_from_map += (*it);
    //                 }
    //             }

    //             down_sample_filter_corner.setInputCloud(laser_cloud_corner_from_map);
    //             down_sample_filter_corner.filter(*laser_cloud_corner_from_map);

    //             if (laser_cloud_corner_from_map->points.size()) {
    //                 m_kdtree_corner_from_map.setInputCloud(laser_cloud_corner_from_map);
    //             }

    //             m_if_mapping_updated_corner = false;
    //             m_mutex_buff_for_matching_corner.lock();
    //             *m_laser_cloud_corner_from_map_last = *laser_cloud_corner_from_map;
    //             m_kdtree_corner_from_map_last = m_kdtree_corner_from_map;
    //             m_mutex_buff_for_matching_corner.unlock();
    //         }
    //     }
    // }

    //不断更新local map corners对应的kdtree_corners, localmap plannes对应的kdtree_plannes
    //matching_mode = 0：用history数据,构建local map corners/plannes
    //matching_mode = 1:在整个corners map中找离上一帧位置(x, y, z) 100m以内的所有points构建
    void update_buff_for_matching() {
        if (m_lastest_pc_matching_refresh_time == m_lastest_pc_reg_time)
            return;

        m_timer.tic("Update buff for matching");
        pcl::VoxelGrid<PointType> down_sample_filter_corner = m_down_sample_filter_corner;   //leaf size: line resolution
        pcl::VoxelGrid<PointType> down_sample_filter_surface = m_down_sample_filter_surface; //leaf size: planne resolution
        down_sample_filter_corner.setLeafSize(m_line_resolution, m_line_resolution, m_line_resolution);
        down_sample_filter_surface.setLeafSize(m_plane_resolution, m_plane_resolution, m_plane_resolution);

        pcl::PointCloud<PointType>::Ptr laser_cloud_corner_from_map(new pcl::PointCloud<PointType>()); //local map corners
        pcl::PointCloud<PointType>::Ptr laser_cloud_surf_from_map(new pcl::PointCloud<PointType>());   //local map plannes
        if (m_matching_mode) { //0                                                                        //0
            pcl::VoxelGrid<PointType> down_sample_filter_corner = m_down_sample_filter_corner;
            pcl::VoxelGrid<PointType> down_sample_filter_surface = m_down_sample_filter_surface;
            std::vector<Points_cloud_map<float>::Mapping_cell_ptr> corner_cell_vec = m_pt_cell_map_corners.find_cells_in_radius(m_t_w_curr, m_maximum_search_range_corner);
            std::vector<Points_cloud_map<float>::Mapping_cell_ptr> plane_cell_vec = m_pt_cell_map_planes.find_cells_in_radius(m_t_w_curr, m_maximum_search_range_surface);
            //当matching_mode = 1时，在整个corners map中找离上一帧位置(x, y, z) 100m以内的所有points所在的cells地址，构建local map corners
            //同理，local map plannes

            int corner_cell_numbers_in_fov = 0;
            int surface_cell_numbers_in_fov = 0;
            pcl::PointCloud<PointType> pc_temp;

            for (size_t i = 0; i < corner_cell_vec.size(); i++) {
                int if_in_fov = if_pt_in_fov(corner_cell_vec[i]->m_center.cast<double>()); //判断点是否在视野内
                if (if_in_fov == 0) {
                    continue;
                }
                corner_cell_numbers_in_fov++;
                down_sample_filter_corner.setInputCloud(corner_cell_vec[i]->get_pointcloud().makeShared());
                down_sample_filter_corner.filter(pc_temp);
                if (m_down_sample_replace) { //1
                    corner_cell_vec[i]->set_pointcloud(pc_temp);
                }
                *laser_cloud_corner_from_map += pc_temp;
            }

            for (size_t i = 0; i < plane_cell_vec.size(); i++) {
                int if_in_fov = if_pt_in_fov(plane_cell_vec[i]->m_center.cast<double>());
                if (if_in_fov == 0) {
                    continue;
                }
                surface_cell_numbers_in_fov++;
                down_sample_filter_surface.setInputCloud(plane_cell_vec[i]->get_pointcloud().makeShared());
                down_sample_filter_surface.filter(pc_temp);
                if (m_down_sample_replace) { //1
                    plane_cell_vec[i]->set_pointcloud(pc_temp);
                }
                *laser_cloud_surf_from_map += pc_temp;
            }
        } else { //用算上当前帧在内，最多保留y400帧 history frames(最top的最oldest), 建立local map corners(kdtree_corners), local map plannes(kdtree_plannes)
            m_mutex_mapping.lock();
            for (auto it = m_laser_cloud_corner_history.begin(); it != m_laser_cloud_corner_history.end(); it++) {
                *laser_cloud_corner_from_map += (*it);
            }

            for (auto it = m_laser_cloud_surface_history.begin(); it != m_laser_cloud_surface_history.end(); it++) {
                *laser_cloud_surf_from_map += (*it);
            }

            m_mutex_mapping.unlock();
        }

        down_sample_filter_corner.setInputCloud(laser_cloud_corner_from_map);
        down_sample_filter_corner.filter(*laser_cloud_corner_from_map);

        down_sample_filter_surface.setInputCloud(laser_cloud_surf_from_map);
        down_sample_filter_surface.filter(*laser_cloud_surf_from_map);

        pcl::KdTreeFLANN<PointType> kdtree_corner_from_map;
        pcl::KdTreeFLANN<PointType> kdtree_surf_from_map;

        if (laser_cloud_corner_from_map->points.size() && laser_cloud_surf_from_map->points.size()) {
            kdtree_corner_from_map.setInputCloud(laser_cloud_corner_from_map);
            kdtree_surf_from_map.setInputCloud(laser_cloud_surf_from_map);
        }

        m_if_mapping_updated_corner = false;
        m_if_mapping_updated_surface = false;

        m_mutex_buff_for_matching_corner.lock();
        *m_laser_cloud_corner_from_map_last = *laser_cloud_corner_from_map;
        m_kdtree_corner_from_map_last = kdtree_corner_from_map;
        m_mutex_buff_for_matching_corner.unlock(); //TODO lock 顺序不对, default: suface.unlock

        m_mutex_buff_for_matching_surface.lock();
        *m_laser_cloud_surf_from_map_last = *laser_cloud_surf_from_map;
        m_kdtree_surf_from_map_last = kdtree_surf_from_map;
        m_mutex_buff_for_matching_surface.unlock(); //TODO lock 顺序不对, default: corner.unlock

        if ((m_lastest_pc_reg_time > m_lastest_pc_matching_refresh_time) || (m_lastest_pc_reg_time < 10)) {
            m_lastest_pc_matching_refresh_time = m_lastest_pc_reg_time;
        }
        // *m_logger_matching_buff.get_ostream() << m_timer.toc_string("Update buff for matching") << std::endl;
    }
    void service_update_buff_for_matching() {
        while (1) {
            //if ( m_if_mapping_updated_corner == false and m_if_mapping_updated_surface == false )
            std::this_thread::sleep_for(std::chrono::nanoseconds(100));
            update_buff_for_matching();
        }
    }

    Laser_mapping() {

        m_laser_cloud_corner_last = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
        m_laser_cloud_surf_last = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
        m_laser_cloud_surround = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

        m_laser_cloud_corner_from_map = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
        m_laser_cloud_surf_from_map = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

        m_laser_cloud_corner_from_map_last = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
        m_laser_cloud_surf_from_map_last = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

        m_laser_cloud_full_res = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

        init_parameters(m_ros_node_handle);

        //livox_corners 每帧的corners, plannes, corners + plannes
        m_sub_laser_cloud_corner_last = m_ros_node_handle.subscribe<sensor_msgs::PointCloud2>("/pc2_corners", 10000, &Laser_mapping::laserCloudCornerLastHandler, this); //default: 10000
        m_sub_laser_cloud_surf_last = m_ros_node_handle.subscribe<sensor_msgs::PointCloud2>("/pc2_surface", 10000, &Laser_mapping::laserCloudSurfLastHandler, this);     //default: 10000
        m_sub_laser_cloud_full_res = m_ros_node_handle.subscribe<sensor_msgs::PointCloud2>("/pc2_full", 10000, &Laser_mapping::laserCloudFullResHandler, this);          //default: 10000

        m_pub_laser_cloud_surround = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 10000);
        //每隔100帧发布当前帧周围1000m范围内降采样后的points

        m_pub_last_corner_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/features_corners", 10000);
        //每一帧corners在map下的points，pose graph 更新前的地图

        m_pub_last_surface_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/features_surface", 10000);
        //每一帧plannes在map下的points，pose graph 更新前的地图

        m_pub_match_corner_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/match_pc_corners", 10000);
        //local map corners

        m_pub_match_surface_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/match_pc_surface", 10000);
        //local map plannes

        m_pub_pc_aft_loop = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/pc_aft_loop_closure", 10000);
        //每有一次闭环用pose graph更新后的poses，对地图的各个小部分进行更新，也就是重新计算在map下的points

        m_pub_debug_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/pc_debug", 10000);
        //topic上的数据为空

        //not used
        m_pub_laser_cloud_map = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 10000);

        m_pub_laser_cloud_full_res = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 10000);
        //每一帧corners + plannes 在map下的points, pose graph 更新前的地图

        m_pub_odom_aft_mapped = m_ros_node_handle.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10000);
        //与local map 匹配得到的每帧的位姿

        //not used
        m_pub_odom_aft_mapped_hight_frec = m_ros_node_handle.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 10000);

        //每隔10帧发布全部与local map 匹配得到的每帧的位姿
        m_pub_laser_aft_mapped_path = m_ros_node_handle.advertise<nav_msgs::Path>("/aft_mapped_path", 10000);

        m_pub_laser_aft_loopclosure_path = m_ros_node_handle.advertise<nav_msgs::Path>("/aft_loopclosure_path", 10000);
        //每有一次闭环就会计算一次pose graph， 然后把计算后的位姿发布到该topic

        m_pt_cell_map_full.set_resolution(m_pt_cell_resolution);
        m_pt_cell_map_full.m_minimum_revisit_threshold = m_para_threshold_cell_revisit;

        m_pt_cell_map_corners.set_resolution(m_pt_cell_resolution);
        m_pt_cell_map_corners.m_minimum_revisit_threshold = m_para_threshold_cell_revisit;

        m_pt_cell_map_planes.set_resolution(m_pt_cell_resolution);
        m_pt_cell_map_planes.m_minimum_revisit_threshold = m_para_threshold_cell_revisit;

        m_keyframe_of_updating_list.push_back(std::make_shared<Maps_keyframe<float>>()); //初始化压入一个对象
        //m_current_keyframe = std::make_shared<Maps_keyframe<float>>();
        screen_out << "Laser_mapping init OK" << endl;
    };

    ~Laser_mapping(){};

    Data_pair* get_data_pair(const double& time_stamp) {
        std::map<double, Data_pair*>::iterator it = m_map_data_pair.find(time_stamp);
        if (it == m_map_data_pair.end()) {
            Data_pair* date_pair_ptr = new Data_pair();
            m_map_data_pair.insert(std::make_pair(time_stamp, date_pair_ptr));
            return date_pair_ptr;
        } else {
            return it->second;
        }
    }

    template <typename T>
    T get_ros_parameter(ros::NodeHandle& nh, const std::string parameter_name, T& parameter, T default_val) {
        nh.param<T>(parameter_name.c_str(), parameter, default_val);
        ENABLE_SCREEN_PRINTF; //当有该语句时, screen_out 也就是 std::cout
        screen_out << "[Laser_mapping_ros_param]: " << parameter_name << " ==> " << parameter << std::endl;
        return parameter;
    }

    // ANCHOR ros_parameter_setting
    void init_parameters(ros::NodeHandle& nh) {

        get_ros_parameter<float>(nh, "feature_extraction/mapping_line_resolution", m_line_resolution, 0.4);
        get_ros_parameter<float>(nh, "feature_extraction/mapping_plane_resolution", m_plane_resolution, 0.8);

        if (m_odom_mode == 1) {
            //m_max_buffer_size = 3e8;
        }

        get_ros_parameter<int>(nh, "common/if_verbose_screen_printf", m_if_verbose_screen_printf, 1);
        get_ros_parameter<int>(nh, "common/odom_mode", m_odom_mode, 0);
        get_ros_parameter<int>(nh, "common/maximum_parallel_thread", m_maximum_parallel_thread, 2);
        get_ros_parameter<int>(nh, "common/if_motion_deblur", if_motion_deblur, 0);
        get_ros_parameter<int>(nh, "common/if_save_to_pcd_files", m_if_save_to_pcd_files, 0);
        get_ros_parameter<int>(nh, "common/if_update_mean_and_cov_incrementally", m_if_maps_incre_update_mean_and_cov, 0);
        get_ros_parameter<int>(nh, "common/threshold_cell_revisit", m_para_threshold_cell_revisit, 5000);

        get_ros_parameter<double>(nh, "optimization/minimum_icp_R_diff", m_minimum_icp_R_diff, 0.01);
        get_ros_parameter<double>(nh, "optimization/minimum_icp_T_diff", m_minimum_icp_T_diff, 0.01);
        get_ros_parameter<int>(nh, "optimization/icp_maximum_iteration", m_para_icp_max_iterations, 20);
        get_ros_parameter<int>(nh, "optimization/ceres_maximum_iteration", m_para_cere_max_iterations, 20);
        get_ros_parameter<int>(nh, "optimization/maximum_residual_blocks", m_para_optimization_maximum_residual_block, 1e5);
        get_ros_parameter<float>(nh, "optimization/max_allow_incre_R", m_para_max_angular_rate, 200.0 / 50.0);
        get_ros_parameter<float>(nh, "optimization/max_allow_incre_T", m_para_max_speed, 100.0 / 50.0);
        get_ros_parameter<float>(nh, "optimization/max_allow_final_cost", m_max_final_cost, 1.0);

        get_ros_parameter<int>(nh, "mapping/init_accumulate_frames", m_mapping_init_accumulate_frames, 50);
        get_ros_parameter<int>(nh, "mapping/maximum_histroy_buffer", m_maximum_history_size, 100);
        get_ros_parameter<int>(nh, "mapping/maximum_mapping_buffer", m_max_buffer_size, 5);
        get_ros_parameter<int>(nh, "mapping/matching_mode", m_matching_mode, 1);
        get_ros_parameter<int>(nh, "mapping/input_downsample_mode", m_if_input_downsample_mode, 1);
        get_ros_parameter<double>(nh, "mapping/maximum_in_fov_angle", m_maximum_in_fov_angle, 30);
        get_ros_parameter<double>(nh, "mapping/maximum_pointcloud_delay_time", m_maximum_pointcloud_delay_time, 0.1);
        // get_ros_parameter<double>(nh, "mapping/maximum_in_fov_angle", m_maximum_in_fov_angle, 30);
        get_ros_parameter<double>(nh, "mapping/maximum_search_range_corner", m_maximum_search_range_corner, 100);
        get_ros_parameter<double>(nh, "mapping/maximum_search_range_surface", m_maximum_search_range_surface, 100);
        get_ros_parameter<double>(nh, "mapping/surround_pointcloud_resolution", m_surround_pointcloud_resolution, 0.5);

        get_ros_parameter<int>(nh, "loop_closure/if_enable_loop_closure", m_loop_closure_if_enable, 0);
        get_ros_parameter<int>(nh, "loop_closure/minimum_keyframe_differen", m_loop_closure_minimum_keyframe_differen, 200);
        get_ros_parameter<float>(nh, "loop_closure/minimum_similarity_linear", m_loop_closure_minimum_similarity_linear, 0.65);
        get_ros_parameter<float>(nh, "loop_closure/minimum_similarity_planar", m_loop_closure_minimum_similarity_planar, 0.95);
        get_ros_parameter<float>(nh, "loop_closure/map_alignment_resolution", m_loop_closure_map_alignment_resolution, 0.2);
        get_ros_parameter<float>(nh, "loop_closure/map_alignment_inlier_threshold", m_loop_closure_map_alignment_inlier_threshold, 0.35);
        get_ros_parameter<int>(nh, "loop_closure/map_alignment_maximum_icp_iteration", m_loop_closure_map_alignment_maximum_icp_iteration, 2);
        get_ros_parameter<int>(nh, "loop_closure/maximum_keyframe_in_waiting_list", m_loop_closure_maximum_keyframe_in_wating_list, 3);
        get_ros_parameter<int>(nh, "loop_closure/scans_of_each_keyframe", m_para_scans_of_each_keyframe, 300);
        get_ros_parameter<int>(nh, "loop_closure/scans_between_two_keyframe", m_para_scans_between_two_keyframe, 100);
        get_ros_parameter<int>(nh, "loop_closure/scene_alignment_maximum_residual_block", m_para_scene_alignments_maximum_residual_block, 5000);
        get_ros_parameter<int>(nh, "loop_closure/if_dump_keyframe_data", m_loop_closure_if_dump_keyframe_data, 0);
        get_ros_parameter<int>(nh, "loop_closure/map_alignment_if_dump_matching_result", m_loop_closure_map_alignment_if_dump_matching_result, 0);

        get_ros_parameter<std::string>(nh, "common/log_save_dir", m_log_save_dir_name, "../");

        m_pt_cell_map_full.set_update_mean_and_cov_incrementally(m_if_maps_incre_update_mean_and_cov);
        //从yaml只设置m_pt_cell_map_full的
        //m_pt_cell_map_corners， m_pt_cell_map_plannes的保持默认状态 m_if_incremental_update_mean_and_cov = 0；

        m_logger_common.set_log_dir(m_log_save_dir_name);
        m_logger_common.init("mapping.log");
        m_logger_timer.set_log_dir(m_log_save_dir_name);
        m_logger_timer.init("timer.log");
        m_logger_matching_buff.set_log_dir(m_log_save_dir_name);
        m_logger_matching_buff.init("match_buff.log");

        get_ros_parameter<std::string>(nh, "common/pcd_save_dir", m_pcd_save_dir_name, std::string("./"));
        get_ros_parameter<std::string>(nh, "common/loop_save_dir", m_loop_save_dir_name, m_pcd_save_dir_name.append("_loop"));
        m_sceene_align.init(m_loop_save_dir_name);

        if (m_if_save_to_pcd_files) { //0
            m_pcl_tools_aftmap.set_save_dir_name(m_pcd_save_dir_name);
            m_pcl_tools_raw.set_save_dir_name(m_pcd_save_dir_name);
        }

        m_logger_pcd.set_log_dir(m_log_save_dir_name);
        m_logger_pcd.init("poses.log");

        LOG_FILE_LINE(m_logger_common);
        *m_logger_common.get_ostream() << m_logger_common.version();

        screen_printf("line resolution %f plane resolution %f \n", m_line_resolution, m_plane_resolution);
        m_logger_common.printf("line resolution %f plane resolution %f \n", m_line_resolution, m_plane_resolution);
        m_down_sample_filter_corner.setLeafSize(m_line_resolution, m_line_resolution, m_line_resolution);
        m_down_sample_filter_surface.setLeafSize(m_plane_resolution, m_plane_resolution, m_plane_resolution);

        m_filter_k_means.setMeanK(m_kmean_filter_count);
        m_filter_k_means.setStddevMulThresh(m_kmean_filter_threshold);
    }

    //把三个回调函数中的laser_data合并起来，压入到 m_queue_avail_data
    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2) {
        std::unique_lock<std::mutex> lock(m_mutex_buf);
        Data_pair* data_pair = get_data_pair(laserCloudCornerLast2->header.stamp.toSec());
        data_pair->add_pc_corner(laserCloudCornerLast2);
        if (data_pair->is_completed()) {
            m_queue_avail_data.push(data_pair);
        }
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2) {
        std::unique_lock<std::mutex> lock(m_mutex_buf);
        Data_pair* data_pair = get_data_pair(laserCloudSurfLast2->header.stamp.toSec());
        data_pair->add_pc_plane(laserCloudSurfLast2);
        if (data_pair->is_completed()) {
            m_queue_avail_data.push(data_pair);
        }
    }

    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) {
        std::unique_lock<std::mutex> lock(m_mutex_buf);
        Data_pair* data_pair = get_data_pair(laserCloudFullRes2->header.stamp.toSec());
        data_pair->add_pc_full(laserCloudFullRes2);
        if (data_pair->is_completed()) {
            m_queue_avail_data.push(data_pair);
        }
    }

    template <typename T, typename TT>
    static void save_mat_to_json_writter(T& writer, const std::string& name, const TT& eigen_mat) {
        writer.Key(name.c_str()); // output a key,
        writer.StartArray();      // Between StartArray()/EndArray(),
        for (size_t i = 0; i < (size_t)(eigen_mat.cols() * eigen_mat.rows()); i++)
            writer.Double(eigen_mat(i));
        writer.EndArray();
    }

    template <typename T, typename TT>
    static void save_quaternion_to_json_writter(T& writer, const std::string& name, const Eigen::Quaternion<TT>& q_curr) {
        writer.Key(name.c_str());
        writer.StartArray();
        writer.Double(q_curr.w());
        writer.Double(q_curr.x());
        writer.Double(q_curr.y());
        writer.Double(q_curr.z());
        writer.EndArray();
    }

    template <typename T, typename TT>
    static void save_data_vec_to_json_writter(T& writer, const std::string& name, TT& data_vec) {
        writer.Key(name.c_str());
        writer.StartArray();
        for (auto it = data_vec.begin(); it != data_vec.end(); it++) {
            writer.Double(*it);
        }
        writer.EndArray();
    }

    void dump_pose_and_regerror(std::string file_name, Eigen::Quaterniond& q_curr,
                                Eigen::Vector3d& t_curr,
                                std::list<double>& reg_err_vec) {
        rapidjson::Document document;
        rapidjson::StringBuffer sb;
        rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
        writer.StartObject();
        writer.SetMaxDecimalPlaces(1000); // like set_precision
        save_quaternion_to_json_writter(writer, "Q", q_curr);
        save_mat_to_json_writter(writer, "T", t_curr);
        save_data_vec_to_json_writter(writer, "Reg_err", reg_err_vec);
        writer.EndObject();
        std::fstream ofs;
        ofs.open(file_name.c_str(), std::ios_base::out);
        if (ofs.is_open()) {
            ofs << std::string(sb.GetString()).c_str();
            ofs.close();
        } else {
            for (int i = 0; i < 109; i++) {
                screen_out << "Write data to file: " << file_name << " error!!!" << std::endl;
            }
        }
    }

    void loop_closure_pub_optimzed_path(const Ceres_pose_graph_3d::MapOfPoses& pose3d_aft_loopclosure) {

        nav_msgs::Odometry odom;
        m_laser_after_loopclosure_path.header.stamp = ros::Time::now();
        m_laser_after_loopclosure_path.header.frame_id = "/camera_init";
        for (auto it = pose3d_aft_loopclosure.begin(); it != pose3d_aft_loopclosure.end(); it++) {
            geometry_msgs::PoseStamped pose_stamp;
            Ceres_pose_graph_3d::Pose3d pose_3d = it->second;
            pose_stamp.pose.orientation.x = pose_3d.q.x();
            pose_stamp.pose.orientation.y = pose_3d.q.y();
            pose_stamp.pose.orientation.z = pose_3d.q.z();
            pose_stamp.pose.orientation.w = pose_3d.q.w();

            pose_stamp.pose.position.x = pose_3d.p(0);
            pose_stamp.pose.position.y = pose_3d.p(1);
            pose_stamp.pose.position.z = pose_3d.p(2);

            pose_stamp.header.frame_id = "/camera_init";

            m_laser_after_loopclosure_path.poses.push_back(pose_stamp);
        }

        m_pub_laser_aft_loopclosure_path.publish(m_laser_after_loopclosure_path);
    }

    //ANCHOR loop_detection
    //线程 不断地检测是否有闭环，有闭环后进行对齐，然后进行pose graph更新每个finished keyframe位姿，更新地图发布到 "/pc_aft_loop_closure" 上
    void service_loop_detection() {
        int last_update_index = 0; //not useful
        sensor_msgs::PointCloud2 ros_laser_cloud_surround;
        pcl::PointCloud<PointType> pt_full; //好像始终为空

        Eigen::Quaterniond q_curr;
        Eigen::Vector3d t_curr;

        std::list<double> reg_error_his;
        std::string json_file_name;
        int curren_frame_idx;

        std::vector<std::shared_ptr<Maps_keyframe<float>>> keyframe_vec; //存放所有的finished keyframes
        Scene_alignment<float> scene_align;

        //(第1处)设置registration params, 其他2处在scene_align里
        scene_align.init(m_loop_save_dir_name);                                                                                  //设置registration params
        scene_align.m_accepted_threshold = m_loop_closure_map_alignment_inlier_threshold;                                        //0.2
        scene_align.m_maximum_icp_iteration = m_loop_closure_map_alignment_maximum_icp_iteration;                                //5
        scene_align.set_downsample_resolution(m_loop_closure_map_alignment_resolution, m_loop_closure_map_alignment_resolution); //0.1
        scene_align.m_para_scene_alignments_maximum_residual_block = m_para_scene_alignments_maximum_residual_block;             //3000
        //计算闭环时，set registration params

        Mapping_refine<PointType> map_rfn; //typedef pcl::PointXYZI PointType;
        map_rfn.set_save_dir(std::string(m_loop_save_dir_name).append("/mapping_refined"));
        map_rfn.set_down_sample_resolution(0.2);

        std::vector<std::string> m_filename_vec;
        std::map<int, std::string> map_file_name;

        Ceres_pose_graph_3d::MapOfPoses poses_before, poses_after; //we add for save all finished keyframes and map 20191211@jxl

        Ceres_pose_graph_3d::MapOfPoses pose3d_map, pose3d_map_ori;
        Ceres_pose_graph_3d::VectorOfPose pose3d_vec;           //pose3d_vec[i]: i号finished keyframe位姿
        Ceres_pose_graph_3d::VectorOfConstraints constrain_vec; //相邻两个finished keyframe间的约束(0-->1, 1-->2, 2-->3...)

        float avail_ratio_plane = 0.05; // 0.05 for 300 scans, 0.15 for 1000 scans
        float avail_ratio_line = 0.03;

        PCL_TOOLS::PCL_point_cloud_to_pcd pcd_saver;
        pcd_saver.set_save_dir_name(std::string(m_loop_save_dir_name).append("/pcd"));

        std::map<int, pcl::PointCloud<PointType>> map_id_pc; //所有finished keyframes
        //key:   finished keyframe index(start from 0)
        //value: finished keyframe 存放的m_accumulated_point_cloud

        int if_end = 0;
        pcl::VoxelGrid<PointType> down_sample_filter;
        m_logger_loop_closure.set_log_dir(m_log_save_dir_name);
        m_logger_loop_closure.init("loop_closure.log");
        down_sample_filter.setLeafSize(m_surround_pointcloud_resolution, m_surround_pointcloud_resolution, m_surround_pointcloud_resolution);

        ros::Rate rate(200);
        while (ros::ok()) { //1
            rate.sleep();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            //m_mutex_dump_full_history.lock();

            if (m_keyframe_need_precession_list.size() == 0) { //no finished keyframes
                continue;
            }

            m_timer.tic("New keyframe");
            q_curr = m_keyframe_need_precession_list.front()->m_pose_q;
            t_curr = m_keyframe_need_precession_list.front()->m_pose_t; //当前finished keyframe 位姿
            // q_curr = m_q_w_curr;
            // t_curr = m_t_w_curr;
            reg_error_his = m_his_reg_error;

            m_keyframe_need_precession_list.front()->update_features_of_each_cells(); //计算每个cell的feature type
            m_keyframe_need_precession_list.front()->analyze();                       //计算finished keyframe对应的line，planne histograms

            keyframe_vec.push_back(m_keyframe_need_precession_list.front()); //当前finished keyframe压入keyframe_vec
            m_mutex_keyframe.lock();
            m_keyframe_need_precession_list.pop_front();
            m_mutex_keyframe.unlock();

            curren_frame_idx = keyframe_vec.back()->m_ending_frame_idx;

            if (1) {
                down_sample_filter.setInputCloud(keyframe_vec.back()->m_accumulated_point_cloud.makeShared());
                down_sample_filter.filter(keyframe_vec.back()->m_accumulated_point_cloud);
            }
            map_id_pc.insert(std::make_pair(map_id_pc.size(), keyframe_vec.back()->m_accumulated_point_cloud));
            //key:   finished keyframe index(start from 0)
            //value: finished keyframe 存放的m_accumulated_point_cloud

            pose3d_vec.push_back(Ceres_pose_graph_3d::Pose3d(q_curr, t_curr)); //压入当前finished keyframe 位姿
            //pose3d_vec[i]: i号finished keyframe位姿

            pose3d_map.insert(std::make_pair(pose3d_map.size(), Ceres_pose_graph_3d::Pose3d(q_curr, t_curr)));
            //key:   finished keyframe index(start from 0)
            //value: finished keyframe 位姿

            poses_before = pose3d_map; // we add
            poses_after = poses_before;

            if (pose3d_vec.size() >= 2) { //已经压入了2个以上finished keyframes
                Ceres_pose_graph_3d::Constraint3d temp_csn;
                Eigen::Vector3d relative_T = pose3d_vec[pose3d_vec.size() - 2].q.inverse() * (t_curr - pose3d_vec[pose3d_vec.size() - 2].p);
                Eigen::Quaterniond relative_Q = pose3d_vec[pose3d_vec.size() - 2].q.inverse() * q_curr;
                //从last finished keyframe 到 curr finished keyframe的delta_r,delta_t

                temp_csn = Ceres_pose_graph_3d::Constraint3d(pose3d_vec.size() - 2, pose3d_vec.size() - 1,
                                                             relative_Q, relative_T);
                constrain_vec.push_back(temp_csn);
            }

            // Save pose  //TODO 可以不用写到disk中
            if (0) {
                json_file_name = std::string(m_loop_save_dir_name).append("/pose_").append(std::to_string(curren_frame_idx)).append(".json");
                dump_pose_and_regerror(json_file_name, q_curr, t_curr, reg_error_his);
            }
            last_update_index = m_current_frame_index;
            m_timer.tic("Find loop");

            std::shared_ptr<Maps_keyframe<float>> last_keyframe = keyframe_vec.back(); //当前finished keyframe

            float ratio_non_zero_plane = last_keyframe->m_ratio_nonzero_plane; //not used
            float ratio_non_zero_line = last_keyframe->m_ratio_nonzero_line;   //not used
            if (m_loop_closure_if_dump_keyframe_data)                          //0
            {
                json_file_name = std::string("keyframe_").append(std::to_string(curren_frame_idx)).append(".json");
                last_keyframe->save_to_file(std::string(m_loop_save_dir_name), json_file_name); // Save keyframe data
                pcd_saver.save_to_pcd_files("pcd", pt_full, curren_frame_idx);                  // Save to pcd files
            }
            map_file_name.insert(std::make_pair(map_file_name.size(), std::string(m_loop_save_dir_name).append("/").append(json_file_name)));
            m_filename_vec.push_back(std::string(m_loop_save_dir_name).append("/").append(json_file_name));
            float sim_plane_res_cv = 0, sim_line_res_cv = 0; //not used

            float sim_plane_res = 0, sim_line_res = 0; //当前finished keyframe与某一个history finished history planne cells相似度，line cells相似度
            float sim_plane_res_roi = 0, sim_line_res_roi = 0;

            m_logger_loop_closure.printf("--- Current_idx = %d, lidar_frame_idx = %d ---\r\n", keyframe_vec.size(), curren_frame_idx);
            m_logger_loop_closure.printf("%s", last_keyframe->get_frame_info().c_str());

            for (size_t his = 0; his < keyframe_vec.size() - 1; his++) { //iterative all history finished keyframes
                if (if_end) {
                    break;
                }
                if (keyframe_vec.size() - his < (size_t)m_loop_closure_minimum_keyframe_differen) { //200
                    continue;
                }

                //keyframe_vec 已经压入了200帧以上， 当前keyframe与距自己200帧以上的每个history keyframes逐一作比较
                float ratio_non_zero_plane_his = keyframe_vec[his]->m_ratio_nonzero_plane;
                float ratio_non_zero_line_his = keyframe_vec[his]->m_ratio_nonzero_line;

                if ((ratio_non_zero_plane_his < avail_ratio_plane) && (ratio_non_zero_line_his < avail_ratio_line)) //history line_cells, planne_cells直方图非零率太低，略过该关键帧
                    continue;

                //两个finished keyframe 中心点超过5m，认为不存在闭环
                if (abs(keyframe_vec[his]->m_roi_range - last_keyframe->m_roi_range) > 15.0) { //TODO default: 5m
                    continue;
                }

                // 匹配的前提条件：
                // 条件1： history keyframes index距离自己当前index >= 200 才比较
                // 条件2： history finished keyframe 的line_cells, planne_cells直方图非零率太低(line < 3 % &&planne < 5 %) ，略过该关键帧
                // 条件3： 两个finished keyframe 中心点不能超过5m，否则，认为不存在闭环
                // 条件4： 相似性条件： line_simi > 0.65 && planne_simi > 0.92 或者 planne_simi > 0.94

                sim_plane_res = last_keyframe->max_similiarity_of_two_image(last_keyframe->m_feature_img_plane, keyframe_vec[his]->m_feature_img_plane);
                sim_line_res = last_keyframe->max_similiarity_of_two_image(last_keyframe->m_feature_img_line, keyframe_vec[his]->m_feature_img_line);
                //计算当前finished keyframe与某一个history[i] finished keyframe的 planne_cells_histograms相似性, line_cells_histograms相似性

                if (((sim_line_res > m_loop_closure_minimum_similarity_linear) && (sim_plane_res > 0.92)) ||
                    (sim_plane_res > m_loop_closure_minimum_similarity_planar)) {
                    // if (0) // Enable check in roi
                    // {
                    //     sim_plane_res_roi = last_keyframe->max_similiarity_of_two_image(last_keyframe->m_feature_img_plane_roi, keyframe_vec[his]->m_feature_img_plane_roi);
                    //     sim_line_res_roi = last_keyframe->max_similiarity_of_two_image(last_keyframe->m_feature_img_line_roi, keyframe_vec[his]->m_feature_img_line_roi);
                    //     if (((sim_plane_res_roi > m_loop_closure_minimum_similarity_linear) && (sim_plane_res > 0.92)) ||
                    //         (sim_line_res_roi > m_loop_closure_minimum_similarity_planar)) {
                    //         m_logger_loop_closure.printf("Range in roi check pass\r\n");
                    //     } else {
                    //         continue;
                    //     }
                    // }
                    if ((last_keyframe->m_set_cell.size() - keyframe_vec[his]->m_set_cell.size()) /
                        (last_keyframe->m_set_cell.size() + keyframe_vec[his]->m_set_cell.size()) * 0.1) {
                        continue;
                    }
                    // scene_align.set_downsample_resolution(m_loop_closure_map_alignment_resolution, m_loop_closure_map_alignment_resolution); //0.1
                    // scene_align.m_para_scene_alignments_maximum_residual_block = m_para_scene_alignments_maximum_residual_block; //3000
                    //我们移到创建的地方一起设置参数
                    double icp_score = scene_align.find_tranfrom_of_two_mappings(last_keyframe,                                         //当前finished keyframe
                                                                                 keyframe_vec[his],                                     //正在比较的history keyframe
                                                                                 m_loop_closure_map_alignment_if_dump_matching_result); //0

                    screen_printf("===============================================\r\n");
                    screen_printf("%s -- %s\r\n", m_filename_vec[keyframe_vec.size() - 1].c_str(), m_filename_vec[his].c_str());
                    screen_printf("Loop match score(smaller and better)= %lf, threshold= %lf\r\n\n\n", scene_align.m_pc_reg.m_inlier_threshold, m_loop_closure_map_alignment_inlier_threshold);
                    screen_printf("%s\r\n", scene_align.m_pc_reg.m_final_opt_summary.BriefReport().c_str());

                    m_logger_loop_closure.printf("===============================================\r\n");
                    m_logger_loop_closure.printf("%s -- %s\r\n", m_filename_vec[keyframe_vec.size() - 1].c_str(), m_filename_vec[his].c_str());
                    m_logger_loop_closure.printf("Loop match score(smaller and better)= %lf, threshold= %lf\r\n\n\n", scene_align.m_pc_reg.m_inlier_threshold, m_loop_closure_map_alignment_inlier_threshold);
                    m_logger_loop_closure.printf("%s\r\n", scene_align.m_pc_reg.m_final_opt_summary.BriefReport().c_str());

                    //匹配度差，该history附近不太可能存在回环，step = 8
                    if (scene_align.m_pc_reg.m_inlier_threshold > m_loop_closure_map_alignment_inlier_threshold * 2) { //0.4
                        his += 8; //TODO 或许间隔太大了  default: 10
                        continue;
                    }

                    //匹配度很好
                    if (scene_align.m_pc_reg.m_inlier_threshold < m_loop_closure_map_alignment_inlier_threshold) { //0.2
                        printf("I believe this is true loop.\r\n");
                        m_logger_loop_closure.printf("I believe this is true loop.\r\n");
                        auto Q_a = pose3d_vec[his].q;
                        auto Q_b = pose3d_vec[pose3d_vec.size() - 1].q;
                        auto T_a = pose3d_vec[his].p;
                        auto T_b = pose3d_vec[pose3d_vec.size() - 1].p;
                        auto ICP_q = scene_align.m_pc_reg.m_q_w_curr; //loop closure时,  curr = incre, 从curr到history TF in map
                        auto ICP_t = scene_align.m_pc_reg.m_t_w_curr;

                        ICP_t = (ICP_q.inverse() * (-ICP_t));
                        ICP_q = ICP_q.inverse(); //取相反方向得到history to curr

                        screen_out << "ICP_q = " << ICP_q.coeffs().transpose() << std::endl;
                        screen_out << "ICP_t = " << ICP_t.transpose() << std::endl;
                        for (int i = 0; i < 10; i++) {
                            screen_out << "-------------------------------------" << std::endl;
                            screen_out << ICP_q.coeffs().transpose() << std::endl;
                            screen_out << ICP_t.transpose() << std::endl;
                        }

                        Ceres_pose_graph_3d::VectorOfConstraints constrain_vec_temp;
                        constrain_vec_temp = constrain_vec;
                        constrain_vec_temp.push_back(
                            Scene_alignment<float>::add_constrain_of_loop(pose3d_vec.size() - 1, //curr_finished_keyframe_index
                                                                          his,                   //history_finished_keyframe_index
                                                                          Q_a, T_a,              //history_q, history_t
                                                                          Q_b, T_b,              //wrong_curr_q, wrong_curr_t
                                                                          ICP_q, ICP_t));        //his2Wrongcurr_q_in_map, his2Wrongcurr_t_in_map
                        //计算wrong_curr到 history的正确约束，也就是correct_curr_to_history位姿

                        std::string path_name = m_loop_save_dir_name;
                        std::string g2o_filename = std::string(path_name).append("/loop.g2o");
                        pose3d_map_ori = pose3d_map;
                        auto temp_pose_3d_map = pose3d_map;

                        Scene_alignment<float>::save_edge_and_vertex_to_g2o(g2o_filename.c_str(), temp_pose_3d_map, constrain_vec_temp);
                        //把每个finished keyframe的poses(histories, wrong_curr), 位姿之间的正确约束(histories-histories, correct_curr2history)写入到g2o文件中

                        Ceres_pose_graph_3d::pose_graph_optimization(temp_pose_3d_map, constrain_vec_temp);
                        //每产生一个闭环约束，就用ceres构建pose_graph, 计算一次每个位姿的解, 更新位姿

                        poses_after = temp_pose_3d_map; //we add

                        ROS_WARN("Before graph, trajectory size= %d, after graph size=%d\n", pose3d_map_ori.size(), temp_pose_3d_map.size());

                        // Ceres_pose_graph_3d::OutputPoses(std::string(path_name).append("/poses_ori.txt"), pose3d_map_ori);  //pose graph 优化前的位姿
                        // Ceres_pose_graph_3d::OutputPoses(std::string(path_name).append("/poses_opm.txt"), temp_pose_3d_map);//优化后的位姿
                        // scene_align.dump_file_name(std::string(path_name).append("/file_name.txt"), map_file_name);

                        loop_closure_pub_optimzed_path(temp_pose_3d_map);
                        //每有一次闭环就会计算一次pose graph， 然后把计算后的位姿发布到 ”aft_loopclosure_path“上

                        for (int pc_idx = (int)map_id_pc.size() - 1; pc_idx >= 0; pc_idx -= 1) { //TODO default -2
                            screen_out << "*** Refine pointcloud, curren idx = " << pc_idx << " ***" << endl;
                            auto refined_pt = map_rfn.refine_pointcloud(map_id_pc, pose3d_map_ori, temp_pose_3d_map, pc_idx, 0);
                            //用pose graph更新后的poses，对地图的各个小部分进行更新，也就是重新计算在map下的points

                            pcl::toROSMsg(refined_pt, ros_laser_cloud_surround);
                            ros_laser_cloud_surround.header.stamp = ros::Time::now();
                            ros_laser_cloud_surround.header.frame_id = "/camera_init";
                            m_pub_pc_aft_loop.publish(ros_laser_cloud_surround);
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }

                        //map_rfn.refine_mapping( path_name, 0 );
                        // if (0) {
                        //     map_rfn.refine_mapping(map_id_pc, pose3d_map_ori, temp_pose_3d_map, 1);
                        //     pcl::toROSMsg(map_rfn.m_pts_aft_refind, ros_laser_cloud_surround);
                        //     ros_laser_cloud_surround.header.stamp = ros::Time::now();
                        //     ros_laser_cloud_surround.header.frame_id = "/camera_init";
                        //     m_pub_pc_aft_loop.publish(ros_laser_cloud_surround);
                        // }
                        if_end = 1;
                        break;
                    }      //end 匹配程度很好
                    else { //匹配程度较好, step = 5
                        his += 5;
                    }
                    if (if_end) {
                        break;
                    }
                } //end 满足相似性潜在条件

                if (if_end) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    break;
                }

            } //end all history finished keyframes

            screen_out << m_timer.toc_string("Find loop") << std::endl;

            // scene_align.dump_file_name(std::string(m_loop_save_dir_name).append("/file_name.txt"), map_file_name);

            if (1) {
                m_timer.tic("Pub surround pts");
                pcl::toROSMsg(pt_full, ros_laser_cloud_surround); ///pt_full始终为空
                ros_laser_cloud_surround.header.stamp = ros::Time::now();
                ros_laser_cloud_surround.header.frame_id = "/camera_init";
                m_pub_debug_pts.publish(ros_laser_cloud_surround);
                screen_out << m_timer.toc_string("Pub surround pts") << std::endl;
            }
            if (if_end) { //退出while循环
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // break; default when there is a loop break while, we comment this line to prevent jump while
            }
             //end while
        }

        //TODO we add, when "ctrl + c" mapping, we save all keyframe poses and map, but this code can not execute, why?
        assert(map_id_pc.size() == poses_before.size());
        assert(map_id_pc.size() == poses_after.size());

        pcl::PointCloud<PointType> map, path;
        for (int idx = (int)map_id_pc.size() - 1; idx >= 0; idx -= 1) {
            auto it = std::next(map_id_pc.begin(), idx);
            if (it == map_id_pc.end()) {
                ROS_ERROR("Something wrong in saving map");
            } else {
                int id = it->first;
                auto pc_in = map_id_pc.find(id)->second.makeShared();

                std::vector<Eigen::Matrix<float, 3, 1>> pts_vec_ori, pts_vec_opm;
                pts_vec_ori = PCL_TOOLS::pcl_pts_to_eigen_pts<float, PointType>(pc_in);

                Ceres_pose_graph_3d::Pose3d pose_ori = poses_before.find(id)->second;
                Ceres_pose_graph_3d::Pose3d pose_opm = poses_after.find(id)->second;
                pts_vec_opm = map_rfn.refine_pts<float, double>(pts_vec_ori,
                                                                pose_ori.q.toRotationMatrix(), pose_ori.p,
                                                                pose_opm.q.toRotationMatrix(), pose_opm.p);
                pcl::PointCloud<PointType> pcl_pts;
                pcl_pts = PCL_TOOLS::eigen_pt_to_pcl_pointcloud<PointType>(pts_vec_opm);
                map += pcl_pts;

                PointType keyframe;
                keyframe.x = pose_opm.p(0);
                keyframe.y = pose_opm.p(1);
                keyframe.z = pose_opm.p(2);
                path.push_back(keyframe);
            }
        }

        //save map to disk
        pcl::io::savePCDFileASCII(m_pcd_save_dir_name + "/jxl_livox_map.pcd", map);
        std::printf("Save all finished keyframes and map done\n\n");
    } //end func

    //线程: 每隔100帧发布当前帧周围1000m范围内降采样后的points
    void service_pub_surround_pts() {
        pcl::VoxelGrid<PointType> down_sample_filter_surface;
        down_sample_filter_surface.setLeafSize(m_surround_pointcloud_resolution, m_surround_pointcloud_resolution, m_surround_pointcloud_resolution);
        pcl::PointCloud<PointType> pc_temp;
        sensor_msgs::PointCloud2 ros_laser_cloud_surround;
        std::this_thread::sleep_for(std::chrono::nanoseconds(10)); //10 ns   microseconds：us

        pcl::PointCloud<PointType>::Ptr laser_cloud_surround(new pcl::PointCloud<PointType>()); //当前帧周围的map cells(corners + plannes)
        laser_cloud_surround->reserve(1e8);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //1000 ms
        int last_update_index = 0;

        ros::Rate rate(200);
        while (ros::ok()) { //1
            rate.sleep();
            while (m_current_frame_index - last_update_index < 100) { //每100帧发布一次
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            }
            last_update_index = m_current_frame_index;
            pcl::PointCloud<PointType> pc_temp;
            laser_cloud_surround->clear();
            if (m_pt_cell_map_full.get_cells_size() == 0)
                continue;

            std::vector<Points_cloud_map<float>::Mapping_cell_ptr> cell_vec = m_pt_cell_map_full.find_cells_in_radius(m_t_w_curr, 1000.0);
            //获得当前帧周围的map cells(corners + plannes)
            for (size_t i = 0; i < cell_vec.size(); i++) {
                if (m_down_sample_replace) { //1
                    down_sample_filter_surface.setInputCloud(cell_vec[i]->get_pointcloud().makeShared());
                    down_sample_filter_surface.filter(pc_temp);
                    //cell_vec[ i ]->set_pointcloud( pc_temp );
                    *laser_cloud_surround += pc_temp;
                } else {
                    *laser_cloud_surround += cell_vec[i]->get_pointcloud();
                }
            }

            if (laser_cloud_surround->points.size()) {
                down_sample_filter_surface.setInputCloud(laser_cloud_surround);
                down_sample_filter_surface.filter(*laser_cloud_surround);
                pcl::toROSMsg(*laser_cloud_surround, ros_laser_cloud_surround);
                ros_laser_cloud_surround.header.stamp = ros::Time::now();
                ros_laser_cloud_surround.header.frame_id = "/camera_init";
                m_pub_laser_cloud_surround.publish(ros_laser_cloud_surround);
            }
            //screen_out << "~~~~~~~~~~~ " << "pub_surround_service, size = " << laser_cloud_surround->points.size()  << " ~~~~~~~~~~~" << endl;
        }
    }

    Eigen::Matrix<double, 3, 1> pcl_pt_to_eigend(PointType& pt) {
        return Eigen::Matrix<double, 3, 1>(pt.x, pt.y, pt.z);
    }

    // not used by author
    // receive odomtry
    // void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
    //     m_mutex_buf.lock();
    //     m_odom_que.push(laserOdometry);
    //     m_mutex_buf.unlock();

    //     // high frequence publish
    //     Eigen::Quaterniond q_wodom_curr;
    //     Eigen::Vector3d t_wodom_curr;
    //     q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
    //     q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
    //     q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
    //     q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
    //     t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
    //     t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
    //     t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

    //     Eigen::Quaterniond q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);
    //     Eigen::Vector3d t_w_curr = Eigen::Vector3d::Zero();

    //     nav_msgs::Odometry odomAftMapped;
    //     odomAftMapped.header.frame_id = "/camera_init";
    //     odomAftMapped.child_frame_id = "/aft_mapped";
    //     odomAftMapped.header.stamp = laserOdometry->header.stamp;
    //     odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
    //     odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
    //     odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
    //     odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
    //     odomAftMapped.pose.pose.position.x = t_w_curr.x();
    //     odomAftMapped.pose.pose.position.y = t_w_curr.y();
    //     odomAftMapped.pose.pose.position.z = t_w_curr.z();
    //     m_pub_odom_aft_mapped_hight_frec.publish(odomAftMapped);
    // }

    void find_min_max_intensity(const pcl::PointCloud<PointType>::Ptr pc_ptr, float& min_I, float& max_I) {
        int pt_size = pc_ptr->size();
        min_I = 10000;
        max_I = -min_I;
        for (int i = 0; i < pt_size; i++) {
            min_I = std::min(pc_ptr->points[i].intensity, min_I);
            max_I = std::max(pc_ptr->points[i].intensity, max_I);
        }
    }

    //计算出插值的比例
    float refine_blur(float in_blur, const float& min_blur, const float& max_blur) {
        return (in_blur - min_blur) / (max_blur - min_blur);
    }

    float compute_fov_angle(const PointType& pt) {
        float sq_xy = sqrt(std::pow(pt.y / pt.x, 2) + std::pow(pt.z / pt.x, 2));
        return atan(sq_xy) * 57.3;
    }

    void init_pointcloud_registration(Point_cloud_registration& pc_reg) {
        //printf_line;
        //pc_reg.m_kdtree_corner_from_map = m_kdtree_corner_from_map;
        //pc_reg.m_kdtree_surf_from_map = m_kdtree_surf_from_map;
        pc_reg.m_logger_common = &m_logger_common;
        pc_reg.m_logger_pcd = &m_logger_pcd;
        pc_reg.m_logger_timer = &m_logger_timer;
        pc_reg.m_timer = &m_timer;
        pc_reg.m_if_motion_deblur = if_motion_deblur;
        pc_reg.m_current_frame_index = m_current_frame_index;                       //从0开始，每来一帧递增1
        pc_reg.m_mapping_init_accumulate_frames = m_mapping_init_accumulate_frames; //mapping/init_accumulate_frames

        pc_reg.m_last_time_stamp = m_last_time_stamp;
        pc_reg.m_para_max_angular_rate = m_para_max_angular_rate; //4.0
        pc_reg.m_para_max_speed = m_para_max_speed;               //2.0
        pc_reg.m_max_final_cost = m_max_final_cost;               //2.0
        pc_reg.m_para_icp_max_iterations = m_para_icp_max_iterations;
        pc_reg.m_para_cere_max_iterations = m_para_cere_max_iterations;
        pc_reg.m_maximum_allow_residual_block = m_para_optimization_maximum_residual_block;
        pc_reg.m_minimum_pt_time_stamp = m_minimum_pt_time_stamp; //上一帧点中最大的时间戳
        pc_reg.m_maximum_pt_time_stamp = m_maximum_pt_time_stamp; //本帧点中最大的时间戳
        pc_reg.m_minimum_icp_R_diff = m_minimum_icp_R_diff;       //0.01
        pc_reg.m_minimum_icp_T_diff = m_minimum_icp_T_diff;       //0.01
        pc_reg.m_q_w_last = m_q_w_curr;                           //上一帧在map下位姿
        pc_reg.m_t_w_last = m_t_w_curr;

        pc_reg.m_q_w_curr = m_q_w_curr; //用上一帧在map下位姿，作为初值。后面registration 计算的就是m_q_w_curr， m_t_w_curr
        pc_reg.m_t_w_curr = m_t_w_curr;

        //printf_line;
    }

    int if_matchbuff_and_pc_sync(float point_cloud_current_timestamp) {
        if (m_lastest_pc_matching_refresh_time < 0)
            return 1;
        if (point_cloud_current_timestamp - m_lastest_pc_matching_refresh_time < m_maximum_pointcloud_delay_time)
            return 1;
        if (m_lastest_pc_reg_time == m_lastest_pc_matching_refresh_time) // All is processed
            return 1;
        screen_printf("*** Current pointcloud timestamp = %.3f, lastest buff timestamp = %.3f, lastest_pc_reg_time = %.3f ***\r\n",
                      point_cloud_current_timestamp,
                      m_lastest_pc_matching_refresh_time,
                      m_lastest_pc_reg_time);
        //cout << "~~~~~~~~~~~~~~~~ Wait sync, " << point_cloud_current_timestamp << ", " << m_lastest_pc_matching_refresh_time << endl;

        return 0;
    }

    //process线程开启的另一个线程
    int process_new_scan() {
        m_timer.tic("Frame process");
        m_timer.tic("Query points for match");

        Common_tools::Timer timer_frame;
        timer_frame.tic();
        pcl::PointCloud<PointType> current_laser_cloud_full, current_laser_cloud_corner_last, current_laser_cloud_surf_last;
        //当前要处理的帧

        pcl::VoxelGrid<PointType> down_sample_filter_corner = m_down_sample_filter_corner;
        pcl::VoxelGrid<PointType> down_sample_filter_surface = m_down_sample_filter_surface;
        pcl::KdTreeFLANN<PointType> kdtree_corner_from_map;
        pcl::KdTreeFLANN<PointType> kdtree_surf_from_map;

        m_mutex_querypointcloud.lock();
        current_laser_cloud_full = *m_laser_cloud_full_res;           //当前帧全部点云
        current_laser_cloud_corner_last = *m_laser_cloud_corner_last; //当前帧corners
        current_laser_cloud_surf_last = *m_laser_cloud_surf_last;     //当前帧plannes

        float min_t, max_t; //本帧数据中最小，最大时间戳
        find_min_max_intensity(current_laser_cloud_full.makeShared(), min_t, max_t);
        //点的intensity存放的是时间戳

        double point_cloud_current_timestamp = min_t;
        if (point_cloud_current_timestamp > m_lastest_pc_income_time) {
            m_lastest_pc_income_time = point_cloud_current_timestamp;
        }
        point_cloud_current_timestamp = m_lastest_pc_income_time;
        //始终让 point_cloud_current_timestamp ， m_lastest_pc_income_time 保持相等

        m_time_odom = m_last_time_stamp;
        m_minimum_pt_time_stamp = m_last_time_stamp;
        m_maximum_pt_time_stamp = max_t;
        m_last_time_stamp = max_t;

        Point_cloud_registration pc_reg;
        init_pointcloud_registration(pc_reg); //local map与当前帧匹配前，set registration params
        m_current_frame_index++;

        double time_odom = ros::Time::now().toSec();
        m_mutex_querypointcloud.unlock();
        screen_printf("****** Before timestamp info = [%.6f, %.6f, %.6f, %.6f ] ****** \r\n", m_minimum_pt_time_stamp, m_maximum_pt_time_stamp, min_t, m_lastest_pc_matching_refresh_time);

        m_timer.tic("Wait sync");
        while (!if_matchbuff_and_pc_sync(point_cloud_current_timestamp)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // *(m_logger_timer.get_ostream()) << m_timer.toc_string("Wait sync") << std::endl;
        screen_printf("****** After timestamp info = [%.6f, %.6f, %.6f, %.6f ] ****** \r\n", m_minimum_pt_time_stamp, m_maximum_pt_time_stamp, min_t, m_lastest_pc_matching_refresh_time);

        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());

        if (m_if_input_downsample_mode) { //1
            down_sample_filter_corner.setInputCloud(current_laser_cloud_corner_last.makeShared());
            down_sample_filter_corner.filter(*laserCloudCornerStack);
            down_sample_filter_surface.setInputCloud(current_laser_cloud_surf_last.makeShared());
            down_sample_filter_surface.filter(*laserCloudSurfStack);
        } else {
            *laserCloudCornerStack = current_laser_cloud_corner_last;
            *laserCloudSurfStack = current_laser_cloud_surf_last;
        }

        int laser_corner_pt_num = laserCloudCornerStack->points.size();
        int laser_surface_pt_num = laserCloudSurfStack->points.size();

        if (m_if_save_to_pcd_files && PCD_SAVE_RAW) { //0
            m_pcl_tools_raw.save_to_pcd_files("raw", current_laser_cloud_full, m_current_frame_index);
        }

        m_q_w_last = m_q_w_curr;
        m_t_w_last = m_t_w_curr;

        pcl::PointCloud<PointType>::Ptr laser_cloud_corner_from_map(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laser_cloud_surf_from_map(new pcl::PointCloud<PointType>());
        int reg_res = 0;

        m_mutex_buff_for_matching_corner.lock();
        *laser_cloud_corner_from_map = *m_laser_cloud_corner_from_map_last; //local map corners
        kdtree_corner_from_map = m_kdtree_corner_from_map_last;             //local_map_kdtree_corners
        m_mutex_buff_for_matching_corner.unlock();                          //TODO lock 顺序不对， default: m_mutex_buff_for_matching_surface.unlock()

        m_mutex_buff_for_matching_surface.lock();
        *laser_cloud_surf_from_map = *m_laser_cloud_surf_from_map_last; //local map plannes
        kdtree_surf_from_map = m_kdtree_surf_from_map_last;             //local_map_kdtree_plannes
        m_mutex_buff_for_matching_surface.unlock();                     //TODO lock 顺序不对，default: m_mutex_buff_for_matching_corner.unlock()

        reg_res = pc_reg.find_out_incremental_transfrom(laser_cloud_corner_from_map, laser_cloud_surf_from_map, //local map
                                                        kdtree_corner_from_map, kdtree_surf_from_map,           //local map kdtree
                                                        laserCloudCornerStack, laserCloudSurfStack);            //当前帧corners, plannes
        //local map 与 当前帧匹配，计算位姿

        screen_out << "Input points size = " << laser_corner_pt_num << ", surface size = " << laser_surface_pt_num << endl;
        screen_out << "Input mapping points size = " << laser_cloud_corner_from_map->points.size() << ", surface size = " << laser_cloud_surf_from_map->points.size() << endl;
        screen_out << "Registration res = " << reg_res << endl;
        if (reg_res == 0) { //计算位姿失败，直接返回
            return 0;
        }
        m_timer.tic("Add new frame");

        PointType pointOri, pointSel;
        pcl::PointCloud<PointType>::Ptr pc_new_feature_corners(new pcl::PointCloud<PointType>()); //当前帧corners在map下坐标
        pcl::PointCloud<PointType>::Ptr pc_new_feature_surface(new pcl::PointCloud<PointType>()); //当前帧plannes在map下坐标
        for (int i = 0; i < laser_corner_pt_num; i++) {
            pc_reg.pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel,
                                       refine_blur(laserCloudCornerStack->points[i].intensity, m_minimum_pt_time_stamp, m_maximum_pt_time_stamp),
                                       g_if_undistore); //pointSel: map下的点
            pc_new_feature_corners->push_back(pointSel);
        }

        for (int i = 0; i < laser_surface_pt_num; i++) {
            pc_reg.pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel,
                                       refine_blur(laserCloudSurfStack->points[i].intensity, m_minimum_pt_time_stamp, m_maximum_pt_time_stamp),
                                       g_if_undistore);
            pc_new_feature_surface->push_back(pointSel);
        }

        down_sample_filter_corner.setInputCloud(pc_new_feature_corners);
        down_sample_filter_corner.filter(*pc_new_feature_corners);
        down_sample_filter_surface.setInputCloud(pc_new_feature_surface);
        down_sample_filter_surface.filter(*pc_new_feature_surface);

        double r_diff = m_q_w_curr.angularDistance(m_last_his_add_q) * 57.3;
        double t_diff = (m_t_w_curr - m_last_his_add_t).norm();

        pc_reg.pointcloudAssociateToMap(current_laser_cloud_full, current_laser_cloud_full, g_if_undistore);
        //转换到map坐标系下

        m_mutex_mapping.lock();

        if (m_laser_cloud_corner_history.size() < (size_t)m_maximum_history_size || //400帧
            (t_diff > history_add_t_step) ||                                        //0
            (r_diff > history_add_angle_step * 57.3)) {                             //0
            m_last_his_add_q = m_q_w_curr;
            m_last_his_add_t = m_t_w_curr;

            m_laser_cloud_corner_history.push_back(*pc_new_feature_corners);
            m_laser_cloud_surface_history.push_back(*pc_new_feature_surface);

            m_mutex_dump_full_history.lock();
            m_laser_cloud_full_history.push_back(current_laser_cloud_full);
            m_his_reg_error.push_back(pc_reg.m_inlier_threshold); //每一帧点云与local map匹配计算出位姿后的cost，the smaller the more certain about calculated pose.
            m_mutex_dump_full_history.unlock();
        } else {
            screen_printf("==== Reject add history, T_norm = %.2f, R_norm = %.2f ====\r\n", t_diff, r_diff);
        }

        screen_out << "m_pt_cell_map_corners.size() = " << m_pt_cell_map_corners.get_cells_size() << endl;
        screen_out << "m_pt_cell_map_planes.size() = " << m_pt_cell_map_planes.get_cells_size() << endl;

        if (m_laser_cloud_corner_history.size() > (size_t)m_maximum_history_size) { //400帧
            (m_laser_cloud_corner_history.front()).clear();
            m_laser_cloud_corner_history.pop_front();
        }

        if (m_laser_cloud_surface_history.size() > (size_t)m_maximum_history_size) { //400帧
            (m_laser_cloud_surface_history.front()).clear();
            m_laser_cloud_surface_history.pop_front();
        }

        if (m_laser_cloud_full_history.size() > (size_t)m_maximum_history_size) { //400帧
            m_mutex_dump_full_history.lock();
            m_laser_cloud_full_history.front().clear();
            m_laser_cloud_full_history.pop_front();
            m_his_reg_error.pop_front();
            m_mutex_dump_full_history.unlock();
        }
        //m_laser_cloud_corner_history, m_laser_cloud_surface_history, m_laser_cloud_full_history
        //算上当前帧在内，保留400帧 history frame

        m_if_mapping_updated_corner = true;
        m_if_mapping_updated_surface = true;

        m_pt_cell_map_corners.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, PointType>(pc_new_feature_corners));
        m_pt_cell_map_planes.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, PointType>(pc_new_feature_surface));
        //把每一帧corners在map下点加入到m_pt_cell_map_corners
        //把每一帧plannes在map下点加入到m_pt_cell_map_planes

        *(m_logger_common.get_ostream()) << "New added regtime " << point_cloud_current_timestamp << endl;
        if ((m_lastest_pc_reg_time < point_cloud_current_timestamp) || (point_cloud_current_timestamp < 10.0)) {
            m_q_w_curr = pc_reg.m_q_w_curr;
            m_t_w_curr = pc_reg.m_t_w_curr;
            m_lastest_pc_reg_time = point_cloud_current_timestamp;
        } else {
            // *(m_logger_common.get_ostream()) << "***** older update, reject update pose *****" << endl;
        }
        // *(m_logger_pcd.get_ostream()) << "--------------------" << endl;
        // m_logger_pcd.printf("Curr_Q = %f,%f,%f,%f\r\n", m_q_w_curr.w(), m_q_w_curr.x(), m_q_w_curr.y(), m_q_w_curr.z());
        // m_logger_pcd.printf("Curr_T = %f,%f,%f\r\n", m_t_w_curr(0), m_t_w_curr(1), m_t_w_curr(2));
        // m_logger_pcd.printf("Incre_Q = %f,%f,%f,%f\r\n", pc_reg.m_q_w_incre.w(), pc_reg.m_q_w_incre.x(), pc_reg.m_q_w_incre.y(), pc_reg.m_q_w_incre.z());
        // m_logger_pcd.printf("Incre_T = %f,%f,%f\r\n", pc_reg.m_t_w_incre(0), pc_reg.m_t_w_incre(1), pc_reg.m_t_w_incre(2));
        // m_logger_pcd.printf("Cos`t=%f,blk_size = %d \r\n", m_final_opt_summary.final_cost, m_final_opt_summary.num_residual_blocks);
        // *(m_logger_pcd.get_ostream()) << m_final_opt_summary.BriefReport() << endl;

        m_mutex_mapping.unlock();

        //在处理第0帧数据时，再开启一个线程
        if (m_thread_match_buff_refresh.size() < (size_t)m_maximum_mapping_buff_thread) { //1 const value
            std::future<void>* m_mapping_refresh_service =
                new std::future<void>(std::async(std::launch::async, &Laser_mapping::service_update_buff_for_matching, this));
            m_thread_match_buff_refresh.push_back(m_mapping_refresh_service);  
        }

        // ANCHOR processiong keyframe
        if (m_loop_closure_if_enable) { //true
            std::set<Points_cloud_map<float>::Mapping_cell_ptr> cell_vec; //cell_vec集合里保存的cells是：在处理该帧点云时，每一个cell中至少有该帧点云中3个点插入进来
            m_pt_cell_map_full.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, PointType>(current_laser_cloud_full.makeShared()),
                                            &cell_vec);
            //把每一帧corners + plannes在map下点加入到m_pt_cell_map_full

            std::unique_lock<std::mutex> lock(m_mutex_keyframe);
            for (auto it = m_keyframe_of_updating_list.begin(); it != m_keyframe_of_updating_list.end(); it++) {
                (*it)->add_cells(cell_vec);
            }
            //把当前帧的points对应的cells地址(有的是新产生的cell,有的是已有的cell)压入到m_keyframe_of_updating_list的每一个关键帧里

            if (m_keyframe_of_updating_list.front()->m_accumulate_frames >= (size_t)m_para_scans_of_each_keyframe) { //finished关键帧里已经插入300帧frame
                m_keyframe_of_updating_list.front()->m_ending_frame_idx = m_current_frame_index;

                m_keyframe_of_updating_list.front()->m_pose_q = m_q_w_curr;
                m_keyframe_of_updating_list.front()->m_pose_t = m_t_w_curr;

                m_keyframe_need_precession_list.push_back(m_keyframe_of_updating_list.front());
                m_keyframe_of_updating_list.pop_front();
            }

            if (m_keyframe_of_updating_list.back()->m_accumulate_frames >= (size_t)m_para_scans_between_two_keyframe) { //100帧
                m_mutex_dump_full_history.lock();

                for (auto it = m_laser_cloud_full_history.begin(); it != m_laser_cloud_full_history.end(); it++) {
                    m_keyframe_of_updating_list.back()->m_accumulated_point_cloud += (*it);
                }

                if (m_keyframe_need_precession_list.size() > m_loop_closure_maximum_keyframe_in_wating_list) { //最多10帧
                    m_keyframe_need_precession_list.pop_front();
                }
                m_laser_cloud_full_history.clear(); //清零m_laser_cloud_full_history
                m_mutex_dump_full_history.unlock();
                m_keyframe_of_updating_list.push_back(std::make_shared<Maps_keyframe<float>>());

                screen_out << "Number of keyframes in update lists: " << m_keyframe_of_updating_list.size() << std::endl;
            }
        } else {
            m_pt_cell_map_full.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, PointType>(current_laser_cloud_full.makeShared()));
        }

        m_mutex_ros_pub.lock();
        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(current_laser_cloud_full, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(time_odom);
        laserCloudFullRes3.header.frame_id = "/camera_init";
        m_pub_laser_cloud_full_res.publish(laserCloudFullRes3); //single_frame_with_pose_tranfromed

        if (PUB_DEBUG_INFO) { //0
            pcl::PointCloud<PointType> pc_feature_pub_corners, pc_feature_pub_surface;
            sensor_msgs::PointCloud2 laserCloudMsg;

            pc_reg.pointcloudAssociateToMap(current_laser_cloud_surf_last, pc_feature_pub_surface, g_if_undistore);
            pcl::toROSMsg(pc_feature_pub_surface, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(time_odom);
            laserCloudMsg.header.frame_id = "/camera_init";
            m_pub_last_surface_pts.publish(laserCloudMsg);

            pc_reg.pointcloudAssociateToMap(current_laser_cloud_corner_last, pc_feature_pub_corners, g_if_undistore);
            pcl::toROSMsg(pc_feature_pub_corners, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(time_odom);
            laserCloudMsg.header.frame_id = "/camera_init";
            m_pub_last_corner_pts.publish(laserCloudMsg);
        }

        if (IF_PUBLISH_SURFACE_AND_CORNER_PTS) { //0
            sensor_msgs::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(*laser_cloud_surf_from_map, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(time_odom);
            laserCloudMsg.header.frame_id = "/camera_init";
            m_pub_match_surface_pts.publish(laserCloudMsg);

            pcl::toROSMsg(*laser_cloud_corner_from_map, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(time_odom);
            laserCloudMsg.header.frame_id = "/camera_init";
            m_pub_match_corner_pts.publish(laserCloudMsg);
        }

        if (m_if_save_to_pcd_files) { //0
            m_pcl_tools_aftmap.save_to_pcd_files("aft_mapp", current_laser_cloud_full, m_current_frame_index);
            *(m_logger_pcd.get_ostream()) << "Save to: " << m_pcl_tools_aftmap.m_save_file_name << endl;
        }

        //printf_line;
        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp = ros::Time().fromSec(time_odom);

        odomAftMapped.pose.pose.orientation.x = m_q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = m_q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = m_q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = m_q_w_curr.w();

        odomAftMapped.pose.pose.position.x = m_t_w_curr.x();
        odomAftMapped.pose.pose.position.y = m_t_w_curr.y();
        odomAftMapped.pose.pose.position.z = m_t_w_curr.z();

        m_pub_odom_aft_mapped.publish(odomAftMapped);

        geometry_msgs::PoseStamped pose_aft_mapped;
        pose_aft_mapped.header = odomAftMapped.header;
        pose_aft_mapped.pose = odomAftMapped.pose.pose;
        m_laser_after_mapped_path.header.stamp = odomAftMapped.header.stamp;
        m_laser_after_mapped_path.header.frame_id = "/camera_init";

        if (m_current_frame_index % 10 == 0) { //每隔10帧发布一次位姿
            m_laser_after_mapped_path.poses.push_back(pose_aft_mapped);
            m_pub_laser_aft_mapped_path.publish(m_laser_after_mapped_path);
        }

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(m_t_w_curr(0), m_t_w_curr(1), m_t_w_curr(2)));

        q.setW(m_q_w_curr.w());
        q.setX(m_q_w_curr.x());
        q.setY(m_q_w_curr.y());
        q.setZ(m_q_w_curr.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

        m_mutex_ros_pub.unlock();
        // *(m_logger_timer.get_ostream()) << m_timer.toc_string("Add new frame") << std::endl;
        // *(m_logger_timer.get_ostream()) << m_timer.toc_string("Frame process") << std::endl;
        //printf_line;
        return 1;
    }

    //单独的一个std线程, 整个程序的入口
    void process() {
        double first_time_stamp = -1;
        m_last_max_blur = 0.0;
        // if (0) {
        //     if (0) {
        //         m_mapping_refresh_service_corner =
        //             new std::future<void>(std::async(std::launch::async, &Laser_mapping::service_update_buff_for_matching_corner, this));
        //         m_mapping_refresh_service_surface =
        //             new std::future<void>(std::async(std::launch::async, &Laser_mapping::service_update_buff_for_matching_surface, this));
        //     } else {
        //         m_mapping_refresh_service =
        //             new std::future<void>(std::async(std::launch::async, &Laser_mapping::service_update_buff_for_matching, this));
        //     }
        // }

        m_service_pub_surround_pts = new std::future<void>(std::async(std::launch::async, &Laser_mapping::service_pub_surround_pts, this));

        if (m_loop_closure_if_enable)
            m_service_loop_detection = new std::future<void>(std::async(std::launch::async, &Laser_mapping::service_loop_detection, this));
            // std::thread loop_detect(&Laser_mapping::service_loop_detection, this); //not work, not create successfully

        timer_all.tic();
        ros::Rate rate(200);
        while (ros::ok()) { //1
            rate.sleep();
            //printf_line;
            // m_logger_common.printf("------------------\r\n");
            // m_logger_timer.printf("------------------\r\n");

            //printf_line;
            while (m_queue_avail_data.empty()) {
                sleep(0.0001);
            }
            m_mutex_buf.lock();
            while (m_queue_avail_data.size() >= (unsigned int)m_max_buffer_size) { //20000000
                ROS_WARN("Drop lidar frame in mapping for real time performance !!!");
                (*m_logger_common.get_ostream()) << "Drop lidar frame in mapping for real time performance !!!" << endl;
                m_queue_avail_data.pop();
            }

            Data_pair* current_data_pair = m_queue_avail_data.front(); //从队列的最开始处理
            m_queue_avail_data.pop();
            m_mutex_buf.unlock();

            m_timer.tic("Prepare to enter thread");

            m_time_pc_corner_past = current_data_pair->m_pc_corner->header.stamp.toSec();

            if (first_time_stamp < 0) {
                first_time_stamp = m_time_pc_corner_past;
            }

            (*m_logger_common.get_ostream()) << "Messgage time stamp = " << m_time_pc_corner_past - first_time_stamp << endl;

            m_mutex_querypointcloud.lock();
            m_laser_cloud_corner_last->clear();
            pcl::fromROSMsg(*(current_data_pair->m_pc_corner), *m_laser_cloud_corner_last);

            m_laser_cloud_surf_last->clear();
            pcl::fromROSMsg(*(current_data_pair->m_pc_plane), *m_laser_cloud_surf_last);

            m_laser_cloud_full_res->clear();
            pcl::fromROSMsg(*(current_data_pair->m_pc_full), *m_laser_cloud_full_res);
            m_mutex_querypointcloud.unlock();

            delete current_data_pair;

            Common_tools::maintain_maximum_thread_pool<std::future<int>*>(m_thread_pool, m_maximum_parallel_thread);
            //    std::list<std::future<int>*> m_thread_pool;

            std::future<int>* thd = new std::future<int>(std::async(std::launch::async, &Laser_mapping::process_new_scan, this));          
            //开启新的线程

            // *(m_logger_timer.get_ostream()) << m_timer.toc_string("Prepare to enter thread") << std::endl;
            m_thread_pool.push_back(thd);

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }
};

#endif // LASER_MAPPING_HPP
