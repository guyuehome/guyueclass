#ifndef PATCHWORK_H
#define PATCHWORK_H


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
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <unistd.h>

// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <Eigen/Eigen>

typedef std::vector<pcl::PointCloud<pcl::PointXYZI>> Ring;
typedef std::vector<Ring> Zone;

    class PatchWork
    {
    private:
        bool _downsample_cloud;
        bool _remove_ground;  
        const float LEAFSIZE=0.1;

        int num_iter_;
        int num_lpr_;
        int num_min_pts_;
        int num_rings_;
        int num_sectors_;
        int num_zones_;
        int num_rings_of_interest_;

        double sensor_height_;
        double th_seeds_;
        double th_dist_;
        double max_range_;
        double min_range_;
        double uprightness_thr_;
        double adaptive_seed_selection_margin_;
        double min_range_z2_; // 12.3625
        double min_range_z3_; // 22.025
        double min_range_z4_; // 41.35

        bool verbose_;

        // For global threshold
        bool   using_global_thr_;
        double global_elevation_thr_;

        float           d_;
        Eigen::MatrixXf        normal_;
        Eigen::VectorXf        singular_values_;
        float           th_dist_d_;
        Eigen::Matrix3f cov_;
        Eigen::Vector4f pc_mean_;
        double          ring_size;
        double          sector_size;

        std::vector<int> num_sectors_each_zone_;
        std::vector<int> num_rings_each_zone_;
        std::vector<double> min_ranges_each_zone;

        std::vector<double> sector_sizes_;
        std::vector<double> ring_sizes_;
        std::vector<double> min_ranges_;
        std::vector<double> elevation_thr_;
        std::vector<double> flatness_thr_;

        std::vector<Zone> ConcentricZoneModel_;

        pcl::PointCloud<pcl::PointXYZI> revert_pc, reject_pc;
        pcl::PointCloud<pcl::PointXYZI> ground_pc_;
        pcl::PointCloud<pcl::PointXYZI> non_ground_pc_;

        pcl::PointCloud<pcl::PointXYZI> regionwise_ground_;
        pcl::PointCloud<pcl::PointXYZI> regionwise_nonground_;

        void extract_initial_seeds_(
            const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
            pcl::PointCloud<pcl::PointXYZI> &init_seeds);
        void estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground);
        void estimate_plane_(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &ground);
        double xy2theta(const double &x, const double &y);
        void initialize_zone(Zone &z, int num_sectors, int num_rings);
        void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);
        void pc2czm(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm);
        void extract_piecewiseground(const int zone_idx,
                                const pcl::PointCloud<pcl::PointXYZI> &src,
                                pcl::PointCloud<pcl::PointXYZI> &dst,
                                pcl::PointCloud<pcl::PointXYZI> &non_groud_dst);
                                

    public:
        PatchWork();        
        ~PatchWork();
        void downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud,
                        float leafsize_x, float leafsize_y, float leafsize_z);
                                
        void clip_cloud_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                            const float clip_x_min, const float clip_x_max,
                            const float clip_y_min, const float clip_y_max,
                            const float clip_z_min, const float clip_z_max, const double in_dist=3);
        void groundfilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &out_ground_points,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &out_groundless_points,
                            float threshold, float floor_max_angle);
        

        void ransac_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_ground,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_unground,
                                  float threshold);      
        void outliersRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);  
        void estimate_ground(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &ground_cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &nonground_cloud);        
        
    
    };
#endif