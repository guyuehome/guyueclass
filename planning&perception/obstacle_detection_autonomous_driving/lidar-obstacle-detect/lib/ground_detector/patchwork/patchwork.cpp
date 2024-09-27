#include "patchwork.h"



/**
 * @brief PatchWork构造函数
 *
 * PatchWork类的构造函数，用于初始化PatchWork对象。
 */
PatchWork::PatchWork(){  
    sensor_height_ = 1.723;
    verbose_ = true;
    num_iter_ = 3;
    num_lpr_ = 20;
    num_min_pts_ = 10;
    th_seeds_ = 0.5;
    th_dist_ = 0.125;
    max_range_ = 80;
    min_range_ = 2.7;
    num_rings_ = 16;
    num_sectors_ = 54;
    uprightness_thr_ = 0.7;
    adaptive_seed_selection_margin_ = -1.1;

    using_global_thr_ = false;
    global_elevation_thr_ = -0.5;

    if (using_global_thr_) {
        //std::cout << "\033[1;33m[Warning] Global elevation threshold is turned on :" << global_elevation_thr_ << "\033[0m" << std::endl;
    } 
    else 
    { 
        //std::cout << "Global thr. is not in use" << std::endl; 
    }

    num_zones_ = 4;
    num_sectors_each_zone_ = {16, 32 ,54, 32};
    num_rings_each_zone_ = {2, 4, 4, 4};
    min_ranges_ = {2.7, 12.3625, 22.025, 41.35};
    elevation_thr_ = {-1.1, -0.59984, -0.4851, -0.4605};
    flatness_thr_ = {0.0, 0.000125, 0.000185, 0.000185};

    // It equals to elevation_thr_.size()/flatness_thr_.size();
    num_rings_of_interest_ = elevation_thr_.size();

    revert_pc.reserve(3000);
    ground_pc_.reserve(3000);
    non_ground_pc_.reserve(3000);
    regionwise_ground_.reserve(3000);
    regionwise_nonground_.reserve(3000);

    min_range_z2_ = min_ranges_[1];
    min_range_z3_ = min_ranges_[2];
    min_range_z4_ = min_ranges_[3];

    min_ranges_   = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
    ring_sizes_   = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                        (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                        (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                        (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
    sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                        2 * M_PI / num_sectors_each_zone_.at(2),
                        2 * M_PI / num_sectors_each_zone_.at(3)};
    //std::cout << "INITIALIZATION COMPLETE" << std::endl;

    for (int iter = 0; iter < num_zones_; ++iter) {
        Zone z;
        initialize_zone(z, num_sectors_each_zone_.at(iter), num_rings_each_zone_.at(iter));
        ConcentricZoneModel_.push_back(z);
    }
}

PatchWork::~PatchWork(){}    

/**
 * @brief 对点云进行下采样
 *
 * 使用 pcl::VoxelGrid 对输入的点云进行下采样，并将结果存储在输出点云中。
 *
 * @param in_cloud 输入点云
 * @param out_cloud 输出点云
 * @param leafsize_x X轴上的叶子大小
 * @param leafsize_y Y轴上的叶子大小
 * @param leafsize_z Z轴上的叶子大小
 */
void PatchWork::downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud,
                               float leafsize_x, float leafsize_y, float leafsize_z)
{
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(in_cloud);
  voxel.setLeafSize(leafsize_x, leafsize_y, leafsize_z);
  voxel.filter(*out_cloud);

}


/**
 * @brief 裁剪点云 ROI
 *
 * 从给定的输入点云中裁剪出一个指定范围的 ROI（Region of Interest），并将结果存储在输出点云中。
 *
 * @param in_cloud 输入点云
 * @param out_cloud 输出点云
 * @param clip_x_min X 轴最小裁剪范围
 * @param clip_x_max X 轴最大裁剪范围
 * @param clip_y_min Y 轴最小裁剪范围
 * @param clip_y_max Y 轴最大裁剪范围
 * @param clip_z_min Z 轴最小裁剪范围
 * @param clip_z_max Z 轴最大裁剪范围
 * @param in_dist 点云内部的距离阈值
 */
void PatchWork::clip_cloud_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                                  const float clip_x_min, const float clip_x_max,
                                  const float clip_y_min, const float clip_y_max,
                                  const float clip_z_min, const float clip_z_max, const double in_dist)

{
  out_cloud->points.clear();
  for (unsigned int i = 0; i < in_cloud->points.size(); i++)
  {
    if (in_cloud->points[i].x >= clip_x_min && in_cloud->points[i].x <= clip_x_max &&
        in_cloud->points[i].y >= clip_y_min && in_cloud->points[i].y <= clip_y_max &&
        in_cloud->points[i].z >= clip_z_min && in_cloud->points[i].z <= clip_z_max &&
        sqrt(pow(in_cloud->points[i].x, 2) + pow(in_cloud->points[i].y, 2)) > in_dist)
    {
      out_cloud->points.push_back(in_cloud->points[i]);
    }
  }
}
/**
 * @brief 对点云进行地面过滤
 *
 * 根据给定的阈值和最大角度，对输入的点云进行地面过滤，
 * 提取出地面点和非地面点，并分别存储在输出点云中。
 *
 * @param in_cloud 输入的点云指针
 * @param out_ground_points 输出的地面点云指针
 * @param out_groundless_points 输出的非地面点云指针
 * @param threshold 距离阈值
 * @param floor_max_angle 地面最大角度
 */
void PatchWork::groundfilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_ground_points,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_groundless_points,
                                  float threshold, float floor_max_angle)

{ 

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr idx(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(floor_max_angle);

  seg.setInputCloud(in_cloud);
  seg.segment(*idx, *coefficients);

  if (idx->indices.size() == 0)
  {
    //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(idx);
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_ground_points);

  extract.setNegative(true);
  extract.filter(*out_groundless_points);  

}

/**
 * @brief RANSAC 滤波器
 *
 * 使用 RANSAC 算法对输入的点云进行滤波，将地面点和非地面点分别输出。
 *
 * @param in_cloud 输入的点云
 * @param out_ground 输出的地面点云
 * @param out_unground 输出的非地面点云
 * @param threshold 距离阈值
 */
void PatchWork::ransac_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_ground,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_unground,
                                  float threshold)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr idx(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  // seg.setMaxIterations(100);
  // seg.setAxis(Eigen::Vector3f(0, 0, 1));
  // seg.setEpsAngle(0.3);
  seg.setInputCloud(in_cloud);
  seg.segment(*idx, *coefficients);

  if (idx->indices.size() == 0)
  {
    //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(idx);
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_ground);
  extract.setNegative(true);
  extract.filter(*out_unground);
}


/**
 * @brief 提取初始种子
 *
 * 从给定的点云中提取初始种子点。
 *
 * @param zone_idx 区域索引
 * @param p_sorted 已排序的点云
 * @param init_seeds 初始种子点云
 */
void PatchWork::extract_initial_seeds_(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    int init_idx = 0;
    // Empirically, adaptive seed selection applying to Z1 is fine
    static double lowest_h_margin_in_close_zone = (sensor_height_ == 0.0)? -0.1 : adaptive_seed_selection_margin_ * sensor_height_;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < lowest_h_margin_in_close_zone) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

/**
 * @brief 估计平面
 *
 * 根据给定的地面点云，估计平面的参数。
 *
 * @param ground 地面点云
 */
void PatchWork::estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground) {
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_         = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
}

/**
 * @brief 初始化区域
 *
 * 初始化给定的区域对象，为其分配指定数量的扇区和环。
 *
 * @param z 区域对象引用
 * @param num_sectors 扇区数量
 * @param num_rings 环数量
 */
void PatchWork::initialize_zone(Zone &z, int num_sectors, int num_rings)
{
    z.clear();
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(1000);
    Ring     ring;
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++) {
        z.emplace_back(ring);
    }

}

/**
 * @brief 清空指定区域内的块
 *
 * 遍历指定区域内的所有块，清空每个块的点集。
 *
 * @param patches 区域内的块集合
 * @param num_sectors 区域内的扇区数
 * @param num_rings 区域内的环数
 */
void PatchWork::flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings)
{
  for (int i = 0; i < num_sectors; i++)
  {
    for (size_t j = 0; j < num_rings; j++)
    {
      if(!patches[j][i].points.empty()) patches[j][i].points.clear();
    }
    
  }
  
}

/**
 * @brief 将笛卡尔坐标系下的坐标转换为极坐标系下的角度
 *
 * 根据给定的笛卡尔坐标系下的 x 和 y 坐标，计算对应的极坐标系下的角度 theta。
 *
 * @param x 笛卡尔坐标系下的 x 坐标
 * @param y 笛卡尔坐标系下的 y 坐标
 *
 * @return 极坐标系下的角度 theta，取值范围为 0 ~ 2 * PI
 */
double PatchWork::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    if (y >= 0) {
        return atan2(y, x); // 1, 2 quadrant
    } else {
        return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    }
}
    
/**
 * @brief 将点云数据从 PC 格式转换为 CZM 格式
 *
 * 将给定的 pcl::PointCloud<pcl::PointXYZI> 类型的点云数据转换为 std::vector<Zone> 类型的 CZM 格式数据。
 *
 * @param src 输入的点云数据
 * @param czm 输出的 CZM 格式数据
 */
void PatchWork::pc2czm(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm) {

    for (auto const &pt : src.points) {
        int    ring_idx, sector_idx;
        double r = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = xy2theta(pt.x, pt.y);

            if (r < min_range_z2_) { // In First rings
                ring_idx   = std::min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx   = std::min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }

    }
}

/**
 * @brief 提取分段地面
 *
 * 从给定的源点云中提取分段地面，并将结果存储在目标点云中。
 *
 * @param zone_idx 区域索引
 * @param src 源点云
 * @param dst 目标点云
 * @param non_ground_dst 非地面点云
 */
void PatchWork::extract_piecewiseground(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src,
        pcl::PointCloud<pcl::PointXYZI> &dst,
        pcl::PointCloud<pcl::PointXYZI> &non_ground_dst) {
    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();
    // 1. set seeds!

    extract_initial_seeds_(zone_idx, src, ground_pc_);
    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++) {
        estimate_plane_(ground_pc_);
        ground_pc_.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int             j      = 0;
        for (auto       &p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int        r      = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    ground_pc_.points.push_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.push_back(src[r]);
                } else {
                    if (i == num_iter_ - 1) {
                        non_ground_dst.push_back(src[r]);
                    }
                }
            }
        }
    }
}

/**
 * @brief 根据点的 z 值进行比较
 *
 * 这是一个模板函数，用于比较两个点的 z 值大小。
 *
 * @param a 第一个点
 * @param b 第二个点
 *
 * @return 如果 a 的 z 值小于 b 的 z 值，返回 true；否则返回 false。
 */
template<typename PointT> bool point_z_cmp(PointT a, PointT b)
{
  return a.z < b.z;
}

/**
 * @brief 估计地面
 *
 * 从给定的点云中估计地面点和非地面点，并将结果分别存储在 ground_cloud 和 non_ground_cloud 中。
 *
 * @param cloud_in 输入的点云
 * @param ground_cloud 输出的地面点云
 * @param non_ground_cloud 输出的非地面点云
 */
void PatchWork::estimate_ground(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &ground_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &non_ground_cloud)
        // double &time_taken) 
{
    // 1.Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    laserCloudIn = *cloud_in;

    // start = ros::Time::now().toSec();

    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_z_cmp<pcl::PointXYZI>);

    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 1.8* sensor_height
    // Sort point according to height, here uses z-axis in default
    auto     it = laserCloudIn.points.begin();
    for (int i  = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1.8 * sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);

    // t1 = ros::Time::now().toSec();
    // 4. pointcloud -> regionwise setting
    for (int k = 0; k < num_zones_; ++k) {
        flush_patches_in_zone(ConcentricZoneModel_[k], num_sectors_each_zone_[k], num_rings_each_zone_[k]);
    }
    pc2czm(laserCloudIn, ConcentricZoneModel_);
    pcl::PointCloud<pcl::PointXYZI> cloud_nonground;
    pcl::PointCloud<pcl::PointXYZI> cloud_out;

    cloud_out.clear();
    cloud_nonground.clear();
    revert_pc.clear();
    reject_pc.clear();

    int      concentric_idx = 0;
    for (int k              = 0; k < num_zones_; ++k) {
        auto          zone     = ConcentricZoneModel_[k];
        for (uint16_t ring_idx = 0; ring_idx < num_rings_each_zone_[k]; ++ring_idx) {
            for (uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone_[k]; ++sector_idx) {
                if (zone[ring_idx][sector_idx].points.size() > num_min_pts_) {
                    extract_piecewiseground(k, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

                    // Status of each patch
                    // used in checking uprightness, elevation, and flatness, respectively
                    const double ground_z_vec       = abs(normal_(2, 0));
                    const double ground_z_elevation = pc_mean_(2, 0);
                    const double surface_variable   =
                                         singular_values_.minCoeff() /
                                         (singular_values_(0) + singular_values_(1) + singular_values_(2));
                    if (ground_z_vec < uprightness_thr_) {
                        // All points are rejected
                        cloud_nonground += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    } else { // satisfy uprightness
                        if (concentric_idx < num_rings_of_interest_) {
                            if (ground_z_elevation > elevation_thr_[ring_idx + 2 * k]) {
                                if (flatness_thr_[ring_idx + 2 * k] > surface_variable) {
                                    if (verbose_) {
                                        // std::cout << "\033[1;36m[Flatness] Recovery operated. Check "
                                        //           << ring_idx + 2 * k
                                        //           << "th param. flatness_thr_: " << flatness_thr_[ring_idx + 2 * k]
                                        //           << " > "
                                        //           << surface_variable << "\033[0m" << std::endl;
                                        // revert_pc += regionwise_ground_;
                                    }
                                    cloud_out += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                } else {
                                    if (verbose_) {
                                        // std::cout << "\033[1;34m[Elevation] Rejection operated. Check "
                                        //           << ring_idx + 2 * k
                                        //           << "th param. of elevation_thr_: " << elevation_thr_[ring_idx + 2 * k]
                                        //           << " < "
                                        //           << ground_z_elevation << "\033[0m" << std::endl;
                                        reject_pc += regionwise_ground_;
                                    }
                                    cloud_nonground += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                }
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        } else {
                            if (using_global_thr_ && (ground_z_elevation > global_elevation_thr_)) {
                                std::cout << "\033[1;33m[Global elevation] " << ground_z_elevation << " > " << global_elevation_thr_
                                     << "\033[0m" << std::endl;
                                cloud_nonground += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        }
                    }

                }
            }
            ++concentric_idx;
        }
    }
    *ground_cloud = cloud_out;
    *non_ground_cloud = cloud_nonground;
}


