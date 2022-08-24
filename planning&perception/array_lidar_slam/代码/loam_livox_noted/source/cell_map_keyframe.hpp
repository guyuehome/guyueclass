// Author: Jiarong Lin          ziv.lin.ljr@gmail.com
#pragma once
#define USE_HASH 1

#include "common.h"
#include "common_tools.h"
#include "pcl_tools.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <Eigen/Eigen>
#include <boost/format.hpp>
#include <iomanip>
#include <map>
#include <math.h>
#include <mutex>
#include <thread>
#include <vector>
#if USE_HASH
#include <unordered_map>
#else
#endif

#define IF_COV_INIT_IDENTITY 0
#define IF_EIGEN_REPLACE 1
#define IF_ENABLE_INCREMENTAL_UPDATE_MEAN_COV 0
#define IF_ENABLE_DUMP_PCL_PTS 0
#define IF_JSON_PRETTY_WRITTER 0

// Resolution of 2d histogram
#define PHI_RESOLUTION 60   //histogram 多少行
#define THETA_RESOLUTION 60 //多少列

typedef pcl::PointXYZI pcl_pt;

// typedef double COMP_TYPE;
   typedef float  COMP_TYPE;

// #define FUNC_T
#define FUNC_T inline

enum Feature_type {
    e_feature_sphere = 0,
    e_feature_line = 1,
    e_feature_plane = 2
};





template <typename DATA_TYPE> //float
class Points_cloud_cell {
public:
    typedef Eigen::Matrix<DATA_TYPE, 3, 1> Eigen_Point;
    typedef Eigen::Matrix<DATA_TYPE, 3, 1> PT_TYPE;

    DATA_TYPE m_resolution;  //cell fixed size
    Eigen::Matrix<DATA_TYPE, 3, 1> m_center; //Cc

    //private:
    Eigen::Matrix<COMP_TYPE, 3, 1> m_xyz_sum; //sum_x, sum_y, sum_z 
    Eigen::Matrix<COMP_TYPE, 3, 1> m_mean, m_mean_last;
    Eigen::Matrix<COMP_TYPE, 3, 3> m_cov_mat, m_cov_mat_last, m_cov_mat_avoid_singularity;
    Eigen::Matrix<COMP_TYPE, 3, 3> m_icov_mat; //covariance inverse

    /** \brief Eigen vectors of voxel covariance matrix */
    Eigen::Matrix<COMP_TYPE, 3, 3> m_eigen_vec; // Eigen vector of covariance matrix
    Eigen::Matrix<COMP_TYPE, 3, 1> m_eigen_val; // Eigen value of covariance values

    int m_last_eigen_decompose_size = 0; //上一次对covariance eigendecompose时，points num
    int m_create_frame_idx = 0;          //在第几帧时创建了该cell
    int m_last_update_frame_idx = 0;     //2种情况下更新：1当cell被创创建时，赋予当时帧号；2当给cell中加入点，且if_treat_revisit=true时，赋予当时帧号

    Feature_type m_feature_type = e_feature_sphere; //cell type

    double m_feature_determine_threshold_line = 1.0 / 3.0;
    double m_feature_determine_threshold_plane = 1.0 / 3.0;

    std::shared_ptr<Points_cloud_cell<DATA_TYPE>> m_previous_visited_cell = nullptr;

    Eigen::Matrix<COMP_TYPE, 3, 1> m_feature_vector; //Cd

public:
    std::vector<PT_TYPE> m_points_vec;    //raw points
    pcl::PointCloud<pcl_pt> m_pcl_pc_vec; //raw points with pcl PointXYZI type

    DATA_TYPE m_cov_det_sqrt; //not used

    bool m_mean_need_update = true;
    bool m_covmat_need_update = true;
    bool m_icovmat_need_update = true;

    size_t m_maximum_points_size = (size_t)1e3; 
    int m_if_incremental_update_mean_and_cov = 0;

    std::shared_ptr<std::mutex> m_mutex_cell;
    double m_last_update_time; //cell update time

    Points_cloud_cell() {
        m_mutex_cell = std::make_shared<std::mutex>();
        clear_data();

        //TODO author doesn't init, we add, copy from this->get_covmat()
        m_mean.setZero();
        if (IF_COV_INIT_IDENTITY) { //0
            m_cov_mat.setIdentity();
        } else {
            m_cov_mat.setZero();
        }       
    };

    ~Points_cloud_cell() {
        m_mutex_cell->try_lock();
        m_mutex_cell->unlock();
        clear_data();
    };
    ADD_SCREEN_PRINTF_OUT_METHOD;


    //把cell的一些属性按照json格式写入到std::string里返回
    FUNC_T std::string to_json_string() {
        // std::unique_lock< std::mutex > lock( *m_mutex_cell );
        get_icovmat(); // update data

        rapidjson::Document document;
        rapidjson::StringBuffer sb;
// See more details in https://github.com/Tencent/rapidjson/blob/master/example/simplewriter/simplewriter.cpp
#if IF_JSON_PRETTY_WRITTER
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
#else
        //rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
        rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
#endif
        writer.StartObject();             // Between StartObject()/EndObject(),
        writer.SetMaxDecimalPlaces(1000); // like set_precision
        int point_number = m_points_vec.size();
        writer.Key("Pt_num");
        writer.Int(point_number);
        writer.Key("Res"); // output a key
        writer.Double(m_resolution);
        Common_tools::save_mat_to_jason_writter(writer, "Center", m_center);
        Common_tools::save_mat_to_jason_writter(writer, "Mean", m_mean);
        if (m_points_vec.size() > 5) {
            Common_tools::save_mat_to_jason_writter(writer, "Cov", m_cov_mat);
            Common_tools::save_mat_to_jason_writter(writer, "Icov", m_icov_mat);
            Common_tools::save_mat_to_jason_writter(writer, "Eig_vec", m_eigen_vec);
            Common_tools::save_mat_to_jason_writter(writer, "Eig_val", m_eigen_val);
        } else {
            Eigen::Matrix<COMP_TYPE, 3, 3> I;
            Eigen::Matrix<COMP_TYPE, 3, 1> Vec3d;
            I.setIdentity();
            Vec3d << 1.0, 1.0, 1.0;
            Common_tools::save_mat_to_jason_writter(writer, "Cov", I);
            Common_tools::save_mat_to_jason_writter(writer, "Icov", I);
            Common_tools::save_mat_to_jason_writter(writer, "Eig_vec", I);
            Common_tools::save_mat_to_jason_writter(writer, "Eig_val", Vec3d);
        }
        writer.Key("Pt_vec");
        writer.SetMaxDecimalPlaces(3);
        writer.StartArray();
        for (unsigned i = 0; i < point_number; i++) {
            writer.Double(m_points_vec[i](0));
            writer.Double(m_points_vec[i](1));
            writer.Double(m_points_vec[i](2));
        }
        writer.EndArray();
        writer.SetMaxDecimalPlaces(1000);

        writer.EndObject();

        return std::string(sb.GetString());
    }

    FUNC_T void save_to_file(const std::string& path = std::string("./"), const std::string& file_name = std::string("")) {
        std::stringstream str_ss;
        Common_tools::create_dir(path);
        if (file_name.compare("") == 0) {
            str_ss << path << "/" << std::setprecision(3)
                   << m_center(0) << "_"
                   << m_center(1) << "_"
                   << m_center(2) << ".json";
        } else {
            str_ss << path << "/" << file_name.c_str();
        }
        std::fstream ofs;
        ofs.open(str_ss.str().c_str(), std::ios_base::out);
        std::cout << "Save to " << str_ss.str();
        if (ofs.is_open()) {
            ofs << to_json_string();
            ofs.close();
            std::cout << " Successful. Number of points = " << m_points_vec.size() << std::endl;
        } else {
            std::cout << " Fail !!!" << std::endl;
        }
    }

    //更新标志，计算points sum_x, sum_y, sum_z
    void set_data_need_update(int if_update_sum = 0) {      
        m_mean_need_update = true;
        m_covmat_need_update = true;
        m_icovmat_need_update = true;
        m_last_eigen_decompose_size = 0;
        if (if_update_sum) {
            m_xyz_sum.setZero();
            for (size_t i = 0; i < m_points_vec.size(); i++) {
                m_xyz_sum += m_points_vec[i].template cast<COMP_TYPE>();
            }
        }
    }

    FUNC_T int get_points_count() {
        return m_points_vec.size();
    }

    FUNC_T Eigen::Matrix<DATA_TYPE, 3, 1> get_center() {
        return m_center.template cast<DATA_TYPE>();
    }

    //返回Cu
    FUNC_T Eigen::Matrix<DATA_TYPE, 3, 1> get_mean() {
        //std::unique_lock<std::mutex> lock(*m_mutex_cell_mean);
        if (m_if_incremental_update_mean_and_cov == false) {
            if (m_mean_need_update) {
                set_data_need_update();
                m_mean = m_xyz_sum / ((DATA_TYPE)(m_points_vec.size()));
            }
            m_mean_need_update = false;
        }
        return m_mean.template cast<DATA_TYPE>();
    }

    //eigendecompose covariance
    FUNC_T void covmat_eig_decompose(int if_force_update = 0) {
        if (if_force_update == 0) //if update is not so large, skip eigendecompse to save time.
        {
            if (m_last_eigen_decompose_size > 1000 && (m_last_eigen_decompose_size / m_points_vec.size() > 0.90)) {
                return;
            }
        }
        m_last_eigen_decompose_size = m_points_vec.size(); 
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<COMP_TYPE, 3, 3>> eigensolver;
        eigensolver.compute(m_cov_mat);
        m_eigen_val = eigensolver.eigenvalues(); //特征值升序排序
        m_eigen_vec = eigensolver.eigenvectors();
    }


    //考虑covariance degenerate情况，返回计算的covariance
    FUNC_T Eigen::Matrix<COMP_TYPE, 3, 3> get_cov_mat_avoid_singularity() // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]
    {
        covmat_eig_decompose();  

        Eigen::Matrix<COMP_TYPE, 3, 3> eigen_val = m_eigen_val.asDiagonal();
        Eigen::Matrix<COMP_TYPE, 3, 3> res_cov_mat;
        COMP_TYPE min_covar_eigvalue;
        COMP_TYPE min_covar_eigvalue_mult_ = 0.01; // pcl: 0.01
        if (!IF_EIGEN_REPLACE) //0
            min_covar_eigvalue_mult_ = 0;

        min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
        if (eigen_val(0, 0) < min_covar_eigvalue) {
            eigen_val(0, 0) = min_covar_eigvalue;

            if (eigen_val(1, 1) < min_covar_eigvalue) {
                eigen_val(1, 1) = min_covar_eigvalue;
            }

            res_cov_mat = m_eigen_vec * eigen_val * m_eigen_vec.inverse();
            if (!std::isfinite(m_cov_mat(0, 0))) {
                res_cov_mat.setIdentity();
            }
        }
        //TODO author doesn't handle else case, we add
        else{
            res_cov_mat = m_cov_mat;
        }

        return res_cov_mat;
    }


    FUNC_T Eigen::Matrix<DATA_TYPE, 3, 3> get_covmat() {
        // std::unique_lock<std::mutex> lock(*m_mutex_cell_cov);
        if (m_covmat_need_update) {
            get_mean();
            size_t pt_size = m_points_vec.size();

            if (pt_size <= 5 || m_if_incremental_update_mean_and_cov == false) { //author default: pt_size < 5
                if (IF_COV_INIT_IDENTITY) {//0
                    m_cov_mat.setIdentity();
                } else {
                    m_cov_mat.setZero();
                }
                if (pt_size <= 2) {
                    return m_cov_mat.template cast<DATA_TYPE>();
                }
                //TODO author compute covariance is wired, not consistent with eq3.
                for (size_t i = 0; i < pt_size; i++) {
                    m_cov_mat = m_cov_mat + (m_points_vec[i] * m_points_vec[i].transpose()).template cast<COMP_TYPE>();
                }
                m_cov_mat -= pt_size * (m_mean * m_mean.transpose());
                m_cov_mat /= (pt_size - 1);
            }

            m_cov_mat_avoid_singularity = get_cov_mat_avoid_singularity();
        }
        m_covmat_need_update = false;
        return m_cov_mat_avoid_singularity.template cast<DATA_TYPE>();
    }

    //get inverse of covariance
    FUNC_T Eigen::Matrix<DATA_TYPE, 3, 3> get_icovmat() {
        if (m_icovmat_need_update) {
            get_covmat();
            m_icov_mat = m_cov_mat.inverse();
            if (!std::isfinite(m_icov_mat(0, 0))) {
                m_icov_mat.setIdentity();
            }
        }
        m_icovmat_need_update = false;
        return m_icov_mat.template cast<DATA_TYPE>();
    }


    FUNC_T pcl::PointCloud<pcl_pt> get_pointcloud() {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        if (!IF_ENABLE_DUMP_PCL_PTS) {//1
            m_pcl_pc_vec = PCL_TOOLS::eigen_pt_to_pcl_pointcloud<pcl_pt, Eigen_Point>(m_points_vec);
        }
        return m_pcl_pc_vec;
    }

    FUNC_T std::vector<PT_TYPE> get_pointcloud_eigen() {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        return m_points_vec;
    }

    FUNC_T void set_pointcloud(pcl::PointCloud<pcl_pt>& pc_in) {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        m_pcl_pc_vec = pc_in;
        m_points_vec = PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl_pt>(pc_in.makeShared());
    }

    FUNC_T void clear_data() {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        m_points_vec.clear();
        m_pcl_pc_vec.clear();
        m_mean.setZero();
        m_xyz_sum.setZero();
        m_cov_mat.setZero();
        set_data_need_update();
    }

    //构造函数
    Points_cloud_cell(const PT_TYPE& cell_center, const DATA_TYPE& res = 1.0) {
        m_mutex_cell = std::make_shared<std::mutex>();
        clear_data();
        m_resolution = res;
        m_maximum_points_size = (int)res * 100.0;
        m_points_vec.reserve(m_maximum_points_size);
        m_center = cell_center;
        //append_pt( cell_center );

        //author doesn't init, we add, copy from this->get_covmat()
        m_mean.setZero();
        if (IF_COV_INIT_IDENTITY) { //0
            m_cov_mat.setIdentity();
        } else {
            m_cov_mat.setZero();
        }
    }

   //给cell中添加一个point
    FUNC_T void append_pt(const PT_TYPE& pt) {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        if (IF_ENABLE_DUMP_PCL_PTS) {//0
            m_pcl_pc_vec.push_back(PCL_TOOLS::eigen_to_pcl_pt<pcl_pt>(pt));
        }
        m_points_vec.push_back(pt);

        if (m_points_vec.size() > m_maximum_points_size) {
            m_maximum_points_size *= 2;
            m_points_vec.reserve(m_maximum_points_size);
        }

        m_xyz_sum = m_xyz_sum + pt.template cast<COMP_TYPE>();

        if (m_if_incremental_update_mean_and_cov) {//在incre模式下，每增加一个点就要立即计算点的mean, cov, sum
            m_mean_last = m_mean;
            m_cov_mat_last = m_cov_mat;

            auto P_new = pt.template cast<COMP_TYPE>();
            size_t N = m_points_vec.size(); //author default: size_t N = m_points_vec.size() - 1
            m_mean = (N * m_mean_last + P_new) / (N + 1);

            if (N > 5) {
                m_cov_mat = ((N - 1) * m_cov_mat_last + (P_new - m_mean_last) * ((P_new - m_mean_last).transpose()) +
                             (N + 1) * (m_mean_last - m_mean) * ((m_mean_last - m_mean).transpose()) +
                             2 * (m_mean_last - m_mean) * ((P_new - m_mean_last).transpose())) /
                            N;
            } else {
                get_covmat();
            }
        }
        m_last_update_time = Common_tools::timer_tic();
        set_data_need_update();
        //m_points_vec.insert(pt);
    }

    //给cell中添加 std::vector<PT_TYPE> pt_vec
    FUNC_T void set_target_pc(const std::vector<PT_TYPE>& pt_vec) {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        // "The three-dimensional normal-distributions transform: an efficient representation for registration, surface analysis, and loop detection"
        // Formulation 6.2 and 6.3
        int pt_size = pt_vec.size();
        clear_data();
        m_points_vec.reserve(m_maximum_points_size);
        for (int i = 0; i < pt_size; i++) {
            append_pt(pt_vec[i]);
        }
        set_data_need_update();
    }

   //计算cell feature vector, 返回cell feature type
    FUNC_T Feature_type determine_feature(int if_recompute = 0) {
        std::unique_lock<std::mutex> lock(*m_mutex_cell);
        if (if_recompute) {
            set_data_need_update(1);
        }

        m_feature_type = e_feature_sphere;
        if (m_points_vec.size() < 5) { //小于5个点的cell，Cd = [0,0,0]
            m_feature_type = e_feature_sphere;
            m_feature_vector << 0, 0, 0;
            return e_feature_sphere;
        }

        get_covmat();

        if ((m_center.template cast<float>() - m_mean.template cast<float>()).norm() > m_resolution * 0.75) {
            m_feature_type = e_feature_sphere;
            m_feature_vector << 0, 0, 0;
            return e_feature_sphere;
        }

        if ((m_eigen_val[1] * m_feature_determine_threshold_plane > m_eigen_val[0])) {
            m_feature_type = e_feature_plane;
            m_feature_vector = m_eigen_vec.block<3, 1>(0, 0);
            return m_feature_type;
        }
        if (m_eigen_val[2] * m_feature_determine_threshold_line > m_eigen_val[1]) {
            m_feature_type = e_feature_line;
            m_feature_vector = m_eigen_vec.block<3, 1>(0, 2);
        }
        return m_feature_type;
    }
};













//================================么得感情的分割线================================

template <typename DATA_TYPE> //float
class Points_cloud_map {
    public:
    typedef Eigen::Matrix<DATA_TYPE, 3, 1> PT_TYPE; 
    typedef Eigen::Matrix<DATA_TYPE, 3, 1> Eigen_Point;

    typedef Points_cloud_cell<DATA_TYPE> Mapping_cell;
    typedef std::shared_ptr<Mapping_cell> Mapping_cell_ptr;

#if USE_HASH //1
    // template <class Key, 
    //           class T,
    //           class Hash = std::hash<Key>,
    //           class KeyEqual = std::equal_to<Key>,
    //           class Allocator = std::allocator<std::pair<const Key, T>>>
    // class unordered_map;
    typedef std::unordered_map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Eigen_pt_hasher, PCL_TOOLS::Eigen_pt_compare> Map_pt_cell;
    typedef typename std::unordered_map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Eigen_pt_hasher, PCL_TOOLS::Eigen_pt_compare>::iterator Map_pt_cell_it;
#else
    // template <
    //     class Key,
    //     class T,
    //     class Compare = std::less<Key>,
    //     class Allocator = std::allocator<std::pair<const Key, T>>>
    // class map;
    typedef std::map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Pt_compare> Map_pt_cell;
    typedef typename std::map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Pt_compare>::iterator Map_pt_cell_it;
#endif

    DATA_TYPE m_x_min, m_x_max;
    DATA_TYPE m_y_min, m_y_max;
    DATA_TYPE m_z_min, m_z_max;
    DATA_TYPE m_resolution; // resolution mean the distance of a cute to its bound, octree's resolution
    Common_tools::Timer m_timer;

    std::vector<Mapping_cell_ptr> m_cell_vec;  //用vector存放每个cell, not useful
    int m_if_incremental_update_mean_and_cov = IF_ENABLE_INCREMENTAL_UPDATE_MEAN_COV; //0

    //std::unique_ptr< std::mutex > m_mapping_mutex;
    std::shared_ptr<std::mutex> m_mapping_mutex;
    std::shared_ptr<std::mutex> m_octotree_mutex;
    std::shared_ptr<std::mutex> m_mutex_addcell;

    std::string m_json_file_name;
    std::vector<std::set<Mapping_cell_ptr>> m_frame_with_cell_index; //not used 

    float m_ratio_nonzero_line, m_ratio_nonzero_plane;
    int m_current_frame_idx;
    int m_minimum_revisit_threshold = std::numeric_limits<int>::max();


    Map_pt_cell     m_map_pt_cell;      //hash table H, map's key is Cc of a cell
    Map_pt_cell_it  m_map_pt_cell_it;

    //to quickly find the specific cells according to its center Cc.
    //The octree O enables to find out all cells located in the specific area of given range.
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> m_octree 
         = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.0001); //Octree O 把cells的center用octree存放起来, 以便于快速建立local map kdtree

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pcl_cells_center 
         = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()); //点云形式存放每个cell的Cc
         
    int m_initialized = false;

    Points_cloud_map() {
        m_mapping_mutex = std::make_shared<std::mutex>();
        m_mutex_addcell = std::make_shared<std::mutex>();
        m_octotree_mutex = std::make_shared<std::mutex>();
        m_x_min = std::numeric_limits<DATA_TYPE>::max();
        m_y_min = std::numeric_limits<DATA_TYPE>::max();
        m_z_min = std::numeric_limits<DATA_TYPE>::max();

        m_x_max = std::numeric_limits<DATA_TYPE>::min();
        m_y_max = std::numeric_limits<DATA_TYPE>::min();
        m_z_max = std::numeric_limits<DATA_TYPE>::min();

        m_pcl_cells_center->reserve(1e5); //初始化的cells个数
        set_resolution(1.0);//octree分辨率
        m_if_verbose_screen_printf = 1;
        m_current_frame_idx = 0;
    }

    ~Points_cloud_map() {
        m_mapping_mutex->try_lock();
        m_mapping_mutex->unlock();

        m_mutex_addcell->try_lock();
        m_mutex_addcell->unlock();

        m_octotree_mutex->try_lock();
        m_octotree_mutex->unlock();
    }

    ADD_SCREEN_PRINTF_OUT_METHOD;

    FUNC_T void set_update_mean_and_cov_incrementally(int flag = 1) {
        m_if_incremental_update_mean_and_cov = flag; //将来用来设置cell更新模式
    }

    FUNC_T int get_cells_size() {
        return m_map_pt_cell.size(); //hash H 大小
    }

    //由一个点确定cell Cc
    FUNC_T PT_TYPE find_cell_center(const PT_TYPE& pt) {
        PT_TYPE cell_center;
        DATA_TYPE box_size = m_resolution * 1.0;
        DATA_TYPE half_of_box_size = m_resolution * 0.5;
        // cell_center( 0 ) = ( std::round( ( pt( 0 ) - m_x_min - half_of_box_size ) / box_size ) ) * box_size + m_x_min + half_of_box_size;
        // cell_center( 1 ) = ( std::round( ( pt( 1 ) - m_y_min - half_of_box_size ) / box_size ) ) * box_size + m_y_min + half_of_box_size;
        // cell_center( 2 ) = ( std::round( ( pt( 2 ) - m_z_min - half_of_box_size ) / box_size ) ) * box_size + m_z_min + half_of_box_size;

        //author not consistent with eq1.
        //TODO cells do not intersect with each other with this definition?
        cell_center(0) = (std::round((pt(0) - half_of_box_size) / box_size)) * box_size + half_of_box_size;
        cell_center(1) = (std::round((pt(1) - half_of_box_size) / box_size)) * box_size + half_of_box_size;
        cell_center(2) = (std::round((pt(2) - half_of_box_size) / box_size)) * box_size + half_of_box_size;
        return cell_center;
    }


    FUNC_T void clear_data() {
        for (Map_pt_cell_it it = m_map_pt_cell.begin(); it != m_map_pt_cell.end(); it++) {
            it->second->clear_data();
        }
        m_map_pt_cell.clear();
        m_cell_vec.clear();
        m_pcl_cells_center->clear();
        m_octree.deleteTree();
    }

    //only called by append_cloud()
    FUNC_T void set_point_cloud(const std::vector<PT_TYPE>& input_pt_vec, std::set<std::shared_ptr<Mapping_cell>>* cell_vec = nullptr) {
        clear_data();
        for (size_t i = 0; i < input_pt_vec.size(); i++) {
            m_x_min = std::min(input_pt_vec[i](0), m_x_min);
            m_y_min = std::min(input_pt_vec[i](1), m_y_min);
            m_z_min = std::min(input_pt_vec[i](2), m_z_min);

            m_x_max = std::max(input_pt_vec[i](0), m_x_max);
            m_y_max = std::max(input_pt_vec[i](1), m_y_max);
            m_z_max = std::max(input_pt_vec[i](2), m_z_max);
        }

        if (cell_vec != nullptr) {
            cell_vec->clear();
        }
        for (size_t i = 0; i < input_pt_vec.size(); i++) {
            std::shared_ptr<Mapping_cell> cell = find_cell(input_pt_vec[i], 1, 1); //找到点应该插入到什么属性的cell中，返回对应的地址
            cell->append_pt(input_pt_vec[i]);
            if (cell_vec != nullptr) {
                cell_vec->insert(cell); //把cell地址压入参数cell_vec
            }
        }

        m_octree.setInputCloud(m_pcl_cells_center); //第一帧点云处理完后，用m_pcl_cells_center创建Octree
        m_octree.addPointsFromInputCloud();
        screen_out << "*** set_point_cloud octree initialization finish ***" << std::endl;

        m_initialized = true;   //把第一帧点云处理完后，设置为true，以后一直为true不改变
        // m_current_frame_idx++;  //TODO 在append_cloud中调用完后，m_current_frame_idx++;， 所以此处应该屏蔽掉
    }


/**
 * @param input_pt_vec 
 * @param cell_vec 
 */

   //called by laser_mapping.hpp
   //保存corners map:                cell_vel = nullptr
   //保存plannes map:                cell_vel = nullptr
   //同时保存corners + plannes map:  cell_vel != nullptr, 结束后cell_vec集合里保存的cells是：在处理该帧点云时，有至少3个点插入到每一个cell中
    FUNC_T void append_cloud(const std::vector<PT_TYPE>& input_pt_vec, std::set<Mapping_cell_ptr>* cell_vec = nullptr) {
        // ENABLE_SCREEN_PRINTF;
        std::map<Mapping_cell_ptr, int> appeared_cell_count; //value: 统计在处理该帧点云时，有多少个点插入到该cell中
        m_timer.tic(__FUNCTION__);
        m_mapping_mutex->lock();
        int current_size = get_cells_size();

        if (current_size == 0) {//第一次给map中添加点云
            set_point_cloud(input_pt_vec, cell_vec);
            m_mapping_mutex->unlock();
        } else {
            m_mapping_mutex->unlock();
            if (cell_vec != nullptr) {
                cell_vec->clear();
            }
            for (size_t i = 0; i < input_pt_vec.size(); i++) {
                Mapping_cell_ptr cell = find_cell(input_pt_vec[i], 1, 1);
                cell->append_pt(input_pt_vec[i]);
                if (cell_vec != nullptr) {
                    auto it = appeared_cell_count.find(cell);
                    if (it != appeared_cell_count.end()) {
                        it->second++;
                    } else {
                        appeared_cell_count.insert(std::make_pair(cell, 1));
                    }
                }
            }

            if (cell_vec != nullptr) {
                for (auto it = appeared_cell_count.begin(); it != appeared_cell_count.end(); it++) {
                    if (it->second >= 3) {//cell中点个数>=3
                        cell_vec->insert(it->first);
                    }
                }
            }
        }

        m_current_frame_idx++;

        screen_out << "Input points size: " << input_pt_vec.size() << ", "
                   << "add cell number: " << get_cells_size() - current_size << ", "
                   << "curren cell number: " << m_map_pt_cell.size() << std::endl;
        screen_out << m_timer.toc_string(__FUNCTION__) << std::endl;
    }


    template <typename T>
    FUNC_T void set_resolution(T resolution) {
        m_resolution = DATA_TYPE(resolution * 0.5);
        screen_out << "Resolution is set as: " << m_resolution << std::endl;
        m_octree.setResolution(m_resolution);
    };


    FUNC_T DATA_TYPE get_resolution() {
        return m_resolution * 2; //TODO: 为什么乘以2？
    }



    FUNC_T Mapping_cell_ptr add_cell(const PT_TYPE& cell_center) {
        std::unique_lock<std::mutex> lock(*m_mutex_addcell);
        Map_pt_cell_it it = m_map_pt_cell.find(cell_center);
        if (it != m_map_pt_cell.end()) { //计算出的cell Cc，hash H中存在
            return it->second;
        }

        //不存在， 创建一个cell
        Mapping_cell_ptr cell = std::make_shared<Mapping_cell>(cell_center, (DATA_TYPE)m_resolution);
        cell->m_create_frame_idx = m_current_frame_idx;
        cell->m_last_update_frame_idx = m_current_frame_idx;
        cell->m_if_incremental_update_mean_and_cov = m_if_incremental_update_mean_and_cov; //设置cell更新方式
        m_map_pt_cell.insert(std::make_pair(cell_center, cell)); 
        //新创建的cell插入到hash H

        if (m_initialized == false) {
            m_pcl_cells_center->push_back(pcl::PointXYZ(cell->m_center(0), cell->m_center(1), cell->m_center(2)));
            //新创建的cell插入到m_pcl_cells_center
        } else { //当第一帧点云被处理完，m_initialized=true, Octree已经被创建
            std::unique_lock<std::mutex> lock(*m_octotree_mutex);
            m_octree.addPointToCloud(pcl::PointXYZ(cell->m_center(0), cell->m_center(1), cell->m_center(2)), m_pcl_cells_center);
            //把Cc压入m_pcl_cells_center同时，也插入到Octree中
        }

        m_cell_vec.push_back(cell); //新创建的cell插入到m_cell_vec
        return cell;
    }


    //找到pt点应该插入到什么属性的cell中，返回对应的地址
    FUNC_T Mapping_cell_ptr find_cell(const PT_TYPE& pt, int if_add = 1, int if_treat_revisit = 0) {
        PT_TYPE cell_center = find_cell_center(pt); //由每个点计算出一个cell Cc

        Map_pt_cell_it it = m_map_pt_cell.find(cell_center);
        if (it == m_map_pt_cell.end()) {//计算出的cell Cc，hash H中不存在
            if (if_add) {
                Mapping_cell_ptr cell_ptr = add_cell(cell_center);
                return cell_ptr;
            } else {
                return nullptr;
            }
        } else {//cell存在
            if (if_treat_revisit) {
                if (m_current_frame_idx - it->second->m_last_update_frame_idx < m_minimum_revisit_threshold) {//在laser_mapping.hpp中通过yaml设置为2000
                    it->second->m_last_update_frame_idx = m_current_frame_idx;
                    return it->second;
                } else {//该cell从上次有point插进来到现在已经过去了2000帧
                    // Avoid confilcts of revisited
                    // ENABLE_SCREEN_PRINTF;
                    // screen_out << "!!!!! Cell revisit, curr_idx = " << m_current_frame_idx << " ,last_idx = " << it->second->m_last_update_frame_idx << std::endl;
                    
                    //重新创建一个cell，中心还是原来的中心
                    Mapping_cell_ptr new_cell = std::make_shared<Mapping_cell>(it->second->get_center(), (DATA_TYPE)m_resolution);
                    m_cell_vec.push_back(new_cell);
                    new_cell->m_previous_visited_cell = it->second; //原来的cel地址保存在新cell的m_previous_visited_cell
                    it->second = new_cell;
                    it->second->m_create_frame_idx = m_current_frame_idx;
                    it->second->m_last_update_frame_idx = m_current_frame_idx;
                    //screen_out << ", find cell addr = " << ( void * ) find_cell( pt ) << std::endl;
                }
            }
            return it->second;
        }
    }

    //在map(Octree)中，找到离pt指定距离内的所有点所在的cells地址
    template <typename T>
    FUNC_T std::vector<Mapping_cell_ptr> find_cells_in_radius(T pt, float searchRadius = 0) {
        std::unique_lock<std::mutex> lock(*m_octotree_mutex);

        std::vector<Mapping_cell_ptr> cells_vec;
        pcl::PointXYZ searchPoint = PCL_TOOLS::eigen_to_pcl_pt<pcl::PointXYZ>(pt);
        std::vector<int> cloudNWRSearch;
        std::vector<float> cloudNWRRadius;

        if (searchRadius == 0) {
            m_octree.radiusSearch(searchPoint, m_resolution, cloudNWRSearch, cloudNWRRadius);
        } else {
            m_octree.radiusSearch(searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);
        }

        PT_TYPE eigen_pt; //cells的center
        for (size_t i = 0; i < cloudNWRSearch.size(); i++) {
            eigen_pt = PCL_TOOLS::pcl_pt_to_eigen<DATA_TYPE>(m_octree.getInputCloud()->points[cloudNWRSearch[i]]);

            cells_vec.push_back(find_cell(eigen_pt)); //(int if_add = 1, int if_treat_revisit = 0)
            //由cell's center 在 H table中找到该cell的地址
        }
        return cells_vec;
    }


    FUNC_T std::string to_json_string(int& avail_cell_size = 0) {
        std::string str;
        str.reserve(m_map_pt_cell.size() * 1e4);
        std::stringstream str_s(str);
        str_s << "[";
        avail_cell_size = 0;
        for (Map_pt_cell_it it = m_map_pt_cell.begin(); it != m_map_pt_cell.end();) {
            Mapping_cell_ptr cell = it->second;

            if (avail_cell_size != 0) {
                str_s << ",";
            }
            str_s << cell->to_json_string();
            avail_cell_size++;

            it++;
            if (it == m_map_pt_cell.end()) {
                break;
            }
        }
        str_s << "]";
        return str_s.str();
    }

    FUNC_T void save_to_file(const std::string& path = std::string("./"), const std::string& file_name = std::string("")) {
        ENABLE_SCREEN_PRINTF;
        std::stringstream str_ss;
        Common_tools::create_dir(path);
        if (file_name.compare("") == 0) {
            str_ss << path << "/" << std::setprecision(3) << "mapping.json";
        } else {
            str_ss << path << "/" << file_name.c_str();
        }
        std::fstream ofs;
        ofs.open(str_ss.str().c_str(), std::ios_base::out);
        screen_out << "Save to " << str_ss.str();
        if (ofs.is_open()) {
            int avail_cell_size = 0;
            ofs << to_json_string(avail_cell_size);
            ofs.close();
            screen_out << " Successful. Number of cell = " << avail_cell_size << std::endl;
        } else {
            screen_out << " Fail !!!" << std::endl;
        }
    }

    template <typename T>
    FUNC_T T* get_json_array(const rapidjson::Document::Array& json_array) {
        T* res_mat = new T[json_array.Size()];
        for (size_t i = 0; i < json_array.Size(); i++) {
            res_mat[i] = (T)json_array[i].GetDouble();
        }
        return res_mat;
    }

    //not used
    // FUNC_T int load_mapping_from_file(const std::string& file_name = std::string("./mapping.json")) {
    //     Common_tools::Timer timer;
    //     timer.tic("Load mapping from json file");
    //     FILE* fp = fopen(file_name.c_str(), "r");
    //     if (fp == nullptr) {
    //         std::cout << "load_mappingg_from_file: " << file_name << " fail!" << std::endl;
    //         return 0;
    //     } else {
    //         m_json_file_name = file_name;
    //         char readBuffer[1 << 16];
    //         rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    //         rapidjson::Document doc;
    //         doc.ParseStream(is);
    //         if (doc.HasParseError()) {
    //             printf("GetParseError, err_code =  %d\n", doc.GetParseError());
    //             return 0;
    //         }

    //         DATA_TYPE* pt_vec_data;
    //         size_t pt_num;
    //         for (unsigned int i = 0; i < doc.Size(); ++i) {
    //             if (i == 0) {
    //                 set_resolution(doc[i]["Res"].GetDouble() * 2.0);
    //             }

    //             Mapping_cell* cell = add_cell(Eigen::Matrix<DATA_TYPE, 3, 1>(get_json_array<DATA_TYPE>(doc[i]["Center"].GetArray())));

    //             cell->m_mean = Eigen::Matrix<COMP_TYPE, 3, 1>(get_json_array<COMP_TYPE>(doc[i]["Mean"].GetArray()));
    //             cell->m_cov_mat = Eigen::Matrix<COMP_TYPE, 3, 3>(get_json_array<COMP_TYPE>(doc[i]["Cov"].GetArray()));
    //             cell->m_icov_mat = Eigen::Matrix<COMP_TYPE, 3, 3>(get_json_array<COMP_TYPE>(doc[i]["Icov"].GetArray()));
    //             cell->m_eigen_vec = Eigen::Matrix<COMP_TYPE, 3, 3>(get_json_array<COMP_TYPE>(doc[i]["Eig_vec"].GetArray()));
    //             cell->m_eigen_val = Eigen::Matrix<COMP_TYPE, 3, 1>(get_json_array<COMP_TYPE>(doc[i]["Eig_val"].GetArray()));

    //             pt_num = doc[i]["Pt_num"].GetInt();
    //             cell->m_points_vec.resize(pt_num);
    //             pt_vec_data = get_json_array<DATA_TYPE>(doc[i]["Pt_vec"].GetArray());
    //             for (size_t pt_idx = 0; pt_idx < pt_num; pt_idx++) {
    //                 cell->m_points_vec[pt_idx] << pt_vec_data[pt_idx * 3 + 0], pt_vec_data[pt_idx * 3 + 1], pt_vec_data[pt_idx * 3 + 2];
    //                 cell->m_xyz_sum = cell->m_xyz_sum + cell->m_points_vec[pt_idx].template cast<COMP_TYPE>();
    //             }
    //             delete pt_vec_data;
    //         }
    //         fclose(fp);

    //         std::cout << timer.toc_string("Load mapping from json file") << std::endl;
    //         return m_map_pt_cell.size();
    //     }
    // }


    //not used
    // FUNC_T std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>> load_pts_from_file(const std::string& file_name = std::string("./mapping.json")) {
    //     Common_tools::Timer timer;
    //     timer.tic("Load points from json file");
    //     FILE* fp = fopen(file_name.c_str(), "r");
    //     std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>> res_vec;
    //     if (fp == nullptr) {
    //         return res_vec;
    //     } else {
    //         m_json_file_name = file_name;
    //         char readBuffer[1 << 16];
    //         rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    //         rapidjson::Document doc;
    //         doc.ParseStream(is);
    //         if (doc.HasParseError()) {
    //             printf("GetParseError, error code = %d\n", doc.GetParseError());
    //             return res_vec;
    //         }

    //         DATA_TYPE* pt_vec_data;
    //         size_t pt_num;

    //         for (unsigned int i = 0; i < doc.Size(); ++i) {
    //             std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>> pt_vec_cell;
    //             pt_num = doc[i]["Pt_num"].GetInt();
    //             pt_vec_cell.resize(pt_num);
    //             pt_vec_data = get_json_array<DATA_TYPE>(doc[i]["Pt_vec"].GetArray());
    //             for (size_t pt_idx = 0; pt_idx < pt_num; pt_idx++) {
    //                 pt_vec_cell[pt_idx] << pt_vec_data[pt_idx * 3 + 0], pt_vec_data[pt_idx * 3 + 1], pt_vec_data[pt_idx * 3 + 2];
    //             }
    //             res_vec.insert(res_vec.end(), pt_vec_cell.begin(), pt_vec_cell.end());
    //         }
    //         fclose(fp);
    //         std::cout << "****** Load point from:" << file_name << "  successful ****** " << std::endl;
    //         std::cout << timer.toc_string("Load points from json file") << std::endl;
    //     }
    //     return res_vec;
    // }

    //template <typename T>
    FUNC_T std::vector<PT_TYPE> query_point_cloud(std::vector<Mapping_cell*>& cell_vec) {
        std::vector<std::vector<PT_TYPE>> pt_vec_vec;
        // pt_vec_vec.reserve(1000);
        pt_vec_vec.reserve(cell_vec.size());

        for (int i = 0; i < cell_vec.size(); i++) {
            pt_vec_vec.push_back(cell_vec[i]->get_pointcloud_eigen());
        }
        return Common_tools::vector_2d_to_1d(pt_vec_vec);
    }
 
    //返回map中满足指定类型的points集合, not used
    // FUNC_T pcl::PointCloud<pcl_pt> extract_specify_points(Feature_type select_type) {
    //     pcl::PointCloud<pcl_pt> res_pt;
    //     int cell_is_selected_type = 0;
    //     for (size_t i = 0; i < m_cell_vec.size(); i++) {
    //         if (m_cell_vec[i]->m_feature_type == select_type) {
    //             cell_is_selected_type++;
    //             res_pt += m_cell_vec[i]->get_pointcloud();
    //         }
    //     }

    //     return res_pt;
    // }
    
    // not used
    // FUNC_T pcl::PointCloud<pcl_pt> get_all_pointcloud() {
    //     pcl::PointCloud<pcl_pt> res_pt;
    //     for (size_t i = 0; i < m_cell_vec.size(); i++) {
    //         res_pt += m_cell_vec[i]->get_pointcloud();
    //     }
    //     return res_pt;
    // }
};















//================================么得感情的分割线================================

template <typename DATA_TYPE> //float
class Maps_keyframe {
    public:
    typedef Eigen::Matrix<DATA_TYPE, 3, 1> PT_TYPE;
    typedef Eigen::Matrix<DATA_TYPE, 3, 1> Eigen_Point;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double m_pose_buffer[7] = {0, 0, 0, 1, 0, 0, 0};

    Eigen::Map<Eigen::Quaterniond> m_pose_q = Eigen::Map<Eigen::Quaterniond>(m_pose_buffer);//关键帧的位姿，是累积的最后一帧的位姿
    Eigen::Map<Eigen::Vector3d> m_pose_t = Eigen::Map<Eigen::Vector3d>(m_pose_buffer + 4);

    typedef Points_cloud_cell<DATA_TYPE> Mapping_cell;
    typedef std::shared_ptr<Mapping_cell> Mapping_cell_ptr;
#if USE_HASH //1
    typedef std::unordered_map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Eigen_pt_hasher, PCL_TOOLS::Eigen_pt_compare> Map_pt_cell;
    typedef typename std::unordered_map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Eigen_pt_hasher, PCL_TOOLS::Eigen_pt_compare>::iterator Map_pt_cell_it;
#else
    typedef std::map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Pt_compare> Map_pt_cell;
    typedef typename std::map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Pt_compare>::iterator Map_pt_cell_it;
#endif
    Map_pt_cell      m_map_pt_cell;   //关键帧的hash H, not useful 
    Map_pt_cell_it   m_map_pt_cell_it;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pcl_cells_center = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    //所有cells center, not useful 

    // ANCHOR cell:resolution
    int scale = 10; //not used
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> m_feature_img_line,     m_feature_img_plane; //对应的histogram
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> m_feature_img_line_roi, m_feature_img_plane_roi;

    std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>> m_feature_vecs_plane, m_feature_vecs_line; //保存finished关键帧所有planne_cells的Cd，line_cells的Cd
    std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>> m_feature_vecs_plane_roi, m_feature_vecs_line_roi; //保存finished关键帧中距离keyframe center 一定距离内，所有planne_cells的Cd， 所有line_cells的Cd

    Eigen::Matrix<float, 3, 3> m_eigen_R, m_eigen_R_roi;
    float m_ratio_nonzero_line, m_ratio_nonzero_plane; //m_feature_img_line,  m_feature_img_plane 2D直方图非零项所占比例


    ADD_SCREEN_PRINTF_OUT_METHOD;
    float m_roi_range; //距离keyframe 中心最远距离的ratio比例是多远
    pcl::PointCloud<PointType> m_accumulated_point_cloud; // The pointcloud sampled from current frame to last frame

    std::set<Mapping_cell_ptr> m_set_cell; //关键帧的cells集合


    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> m_octree    //not useful
          = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.0001);

    unsigned int m_last_update_feature_cells_number = 0; //TODO author doesn't initialize zero, we add

    unsigned int m_accumulate_frames = 0; //积累了多少帧points

    pcl::PointCloud<pcl_pt> m_pcl_snap_shot_line, m_pcl_snap_shot_plane;

    int m_ending_frame_idx;//该关键帧finished时的帧index

    std::shared_ptr<std::mutex> m_keyframe_idx;

    Maps_keyframe() {
        m_keyframe_idx = std::make_shared<std::mutex>();
        m_accumulated_point_cloud.reserve(5000 * 500 / 3); //each frame have 5000/3 pts, about maximum 500 frame of accumulation.
    }

    ~Maps_keyframe(){};

    void make_index_in_matrix_range(int& idx, int maximum_range) {
        if (idx < 0) {
            std::cout << "Idx < 0 !!!" << std::endl;
            idx = 0;
        }
        if (idx >= maximum_range) {
            std::cout << "Idx >= maximum_range !!!" << std::endl;
            idx = maximum_range - 1;
        }
    }

    //ANCHOR cell::feature_direction
    template <typename T>
    FUNC_T void feature_direction(Eigen::Matrix<T, 3, 1>& vec_3d, int& phi_idx, int& theta_idx) {
        if (vec_3d[0] < 0) {  //TODO 为什么要对 x < 0的，要乘以-1？ 论文中说只对 x > 0 的cell处理
            vec_3d *= (-1.0); 
        }
        int phi_res = PHI_RESOLUTION;
        int theta_res = THETA_RESOLUTION;
        double phi_step = M_PI / phi_res;
        double theta_step = M_PI / theta_res;
        double phi = atan2(vec_3d[1], vec_3d[0]) + M_PI / 2;
        double theta = asin(vec_3d[2]) + M_PI / 2;

        phi_idx = (std::floor(phi / phi_step));
        theta_idx = (std::floor(theta / theta_step));
        make_index_in_matrix_range(phi_idx, PHI_RESOLUTION);
        make_index_in_matrix_range(theta_idx, THETA_RESOLUTION);
    }

    //Maps_keyframe类,  not used by author
    // FUNC_T Mapping_cell_ptr find_cell(const PT_TYPE& pt, int if_add = 1) {
    //     Map_pt_cell_it it = m_map_pt_cell.find(pt);
    //     if (it == m_map_pt_cell.end()) {
    //         return nullptr;
    //     } else {
    //         return it->second;
    //     }
    // }

    // Maps_keyframe类, not used by author
    // template <typename T>
    // FUNC_T std::vector<Mapping_cell_ptr> find_cells_in_radius(T pt, float searchRadius) {
    //     std::vector<Mapping_cell_ptr> cells_vec;
    //     pcl::PointXYZ searchPoint = PCL_TOOLS::eigen_to_pcl_pt<pcl::PointXYZ>(pt);
    //     std::vector<int> cloudNWRSearch;
    //     std::vector<float> cloudNWRRadius;
    //     printf_line;

    //     m_octree.radiusSearch(searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);

    //     PT_TYPE eigen_pt;
    //     for (size_t i = 0; i < cloudNWRSearch.size(); i++) {
    //         eigen_pt = PCL_TOOLS::pcl_pt_to_eigen<DATA_TYPE>(m_octree.getInputCloud()->points[cloudNWRSearch[i]]);
    //         cells_vec.push_back(find_cell(eigen_pt));
    //     }

    //     return cells_vec;
    // }

    template <typename T>
    FUNC_T static void refine_feature_img(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& feature_img) {
        int rows = feature_img.rows();
        int cols = feature_img.cols();
        //printf( "Rows = %d, cols = %d\r\n", rows, cols );
        if (feature_img.row(0).maxCoeff() < feature_img.row(rows - 1).maxCoeff()) {
            feature_img = feature_img.colwise().reverse().eval();
        }

        if ((feature_img.block(0, 0, 2, round(cols / 2))).maxCoeff() < (feature_img.block(0, round(cols / 2), 2, round(cols / 2))).maxCoeff()) {
            feature_img = feature_img.rowwise().reverse().eval();
        }
    }

    FUNC_T static float ratio_of_nonzero_in_img(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img) {
        int count = 0;
        for (int i = 0; i < img.rows(); i++) {
            for (int j = 0; j < img.cols(); j++)
                if (img(i, j) >= 1.0)
                    count++;
        }
        return (float)(count) / (img.rows() * img.cols());
    }

    // ANCHOR keyframe::max_similiarity_of_two_image
    FUNC_T static float max_similiarity_of_two_image(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_a, 
                                                     const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_b, 
                                                     float minimum_zero_ratio = 0.00) {
        if (ratio_of_nonzero_in_img(img_a) < minimum_zero_ratio) {
            return 0;
        }

        if (ratio_of_nonzero_in_img(img_b) < minimum_zero_ratio) {
            return 0;
        }
        size_t cols = img_a.cols();
        size_t rows = img_a.rows();
        float max_res = -0;

        cv::Mat hist_a, hist_b; //not used
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> img_b_roi;
        img_b_roi.resize(rows, cols);
        float res = 0;
        // img_b_roi = img_b;
        if (0) {
            for (size_t i = 0; i < rows * 1.0; i++) {
                //Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic > img_b_roi = img_b.block( i, 0, ( int ) std::round( rows / 2 ), cols );
                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> img_b_roi_up = img_b.block(i, 0, rows - i, cols);
                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> img_b_roi_down = img_b.block(0, 0, i, cols);

                img_b_roi << img_b_roi_up, img_b_roi_down;

                res = similiarity_of_two_image_opencv(img_a, img_b_roi);

                if (fabs(res) > fabs(max_res))
                    max_res = fabs(res);
            }
        } else {
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> res_img;
            res_img = add_padding_to_feature_image(img_b, PHI_RESOLUTION * 0.5, THETA_RESOLUTION * 0.5);
            max_res = max_similiarity_of_two_image_opencv(img_a, res_img);//res_img: (rows+60)*(cols+60)
        }

        //std::cout << hist_a + hist_b << std::endl;
        return max_res;
    }



    FUNC_T static float similiarity_of_two_image_opencv(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_a, 
                          const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_b, 
        int method = CV_COMP_CORREL) {
        cv::Mat hist_a, hist_b;
        cv::eigen2cv(img_a, hist_a);
        cv::eigen2cv(img_b, hist_b);
        return cv::compareHist(hist_a, hist_b, method); // https://docs.opencv.org/2.4/modules/imgproc/doc/histograms.html?highlight=comparehist
    }
    // Histogram comparison methods
    // enum {
    //     CV_COMP_CORREL = 0,                   Correlation
    //     CV_COMP_CHISQR = 1,                   Chi-Square
    //     CV_COMP_INTERSECT = 2,                Intersection
    //     CV_COMP_BHATTACHARYYA = 3,            Bhattacharyya distance
    //     CV_COMP_HELLINGER = CV_COMP_BHATTACHARYYA, 
    //     CV_COMP_CHISQR_ALT = 4,
    //     CV_COMP_KL_DIV = 5
    // };
    FUNC_T static float max_similiarity_of_two_image_opencv(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_a,
                                                            const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_b, 
                                                            int method = CV_COMP_CORREL) {
        cv::Mat hist_a, hist_b;
        int cols = img_a.cols();
        int rows = img_a.rows();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> img_a_roi = img_a.block(0, 0, rows, cols);
        cv::eigen2cv(img_a_roi, hist_a);
        cv::eigen2cv(img_b, hist_b);
        cv::Mat result;
        cv::matchTemplate(hist_b, hist_a, result, CV_TM_CCORR_NORMED);
        double minVal;
        double maxVal;
        cv::Point minLoc;
        cv::Point maxLoc;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
        return maxVal;
        //return cv::compareHist( hist_a, hist_b, method ); // https://docs.opencv.org/2.4/modules/imgproc/doc/histograms.html?highlight=comparehist
    }
 
    //called by finished keyframe
    //计算每个cell的feature type
    FUNC_T void update_features_of_each_cells(int if_recompute = 0) {
        if (m_set_cell.size() != m_last_update_feature_cells_number) {
            for (auto it = m_set_cell.begin(); it != m_set_cell.end(); it++) {
                (*it)->determine_feature(if_recompute);
            }
        }
        m_last_update_feature_cells_number = m_set_cell.size();
    }

    //把每一帧的points对应的cells地址(有的是新产生的cells,有的是已有的cells)压入到m_keyframe_of_updating_list的每一个关键帧里
    //把传进来的cells压入到cells集合， 压入到hash H
    FUNC_T void add_cells(const std::set<Mapping_cell_ptr>& cells_vec) {
        unsigned int last_cell_numbers = 0;
        for (typename std::set<Mapping_cell_ptr>::iterator it = cells_vec.begin(); it != cells_vec.end(); it++) {
            m_set_cell.insert(*(it));//加入到cells集合
            m_map_pt_cell.insert(std::make_pair((*(it))->get_center(), *it)); //加入到hash H
            if (last_cell_numbers != m_set_cell.size()) // New different cell
            {
                if (last_cell_numbers == 0) { //作者只压入第一个cell的Cc到cells center,以前遗留的代码见https://github.com/hku-mars/loam_livox/issues/36
                    m_pcl_cells_center->push_back(pcl::PointXYZ((*it)->m_center(0), (*it)->m_center(1), (*it)->m_center(2)));
                }
                last_cell_numbers = m_set_cell.size();
                
            }
        }   
        m_accumulate_frames++;
    }

    //Maps_keyframe 类
    FUNC_T pcl::PointCloud<pcl_pt> extract_specify_points(Feature_type select_type) {
        pcl::PointCloud<pcl_pt> res_pt;
        int cell_is_selected_type = 0;
        for (auto it = m_set_cell.begin(); it != m_set_cell.end(); it++) {
            if ((*it)->m_feature_type == select_type) {
                cell_is_selected_type++;
                res_pt += (*it)->get_pointcloud();
            }
        }

        // screen_out << "Type is " << ( int ) ( select_type ) << ", total is "<< m_set_cell.size()
        //            << ", cell of type: " << cell_is_selected_type << " , size of pts is " << res_pt.points.size() << std::endl;

        return res_pt;
    }

    FUNC_T pcl::PointCloud<pcl_pt> get_all_pointcloud() {
        pcl::PointCloud<pcl_pt> res_pt;
        for (auto it = m_set_cell.begin(); it != m_set_cell.end(); it++) {
            res_pt += (*it)->get_pointcloud();
        }
        return res_pt;
    }

    //计算keyframe所有cells组成的中心
    FUNC_T PT_TYPE get_center() {
        PT_TYPE cell_center;
        cell_center.setZero();
        for (auto it = m_set_cell.begin(); it != m_set_cell.end(); it++) {
            cell_center += (*(it))->get_center();
        }
        cell_center *= (1.0 / (float)m_set_cell.size());
        return cell_center;
    }

    //计算finished keyframe的cells 中心，计算每个cell center与其差保存在 err_vec, 返回距离keyframe 中心最远距离的ratio比例是多远
    FUNC_T float get_ratio_range_of_cell(PT_TYPE& cell_center = PT_TYPE(0, 0, 0), 
                                         float ratio = 0.8, std::vector<PT_TYPE>* err_vec = nullptr) {
        cell_center = get_center();
        std::set<float> dis_vec; //默认升序，距离从小到大排列
        for (auto it = m_set_cell.begin(); it != m_set_cell.end(); it++) {
            PT_TYPE dis = (*(it))->get_center() - cell_center;
            if (err_vec != nullptr) {
                err_vec->push_back(dis);
            }
            dis_vec.insert((float)dis.norm());
        }
        // https://stackoverflow.com/questions/1033089/can-i-increment-an-iterator-by-just-adding-a-number
        return *std::next(dis_vec.begin(), std::ceil((dis_vec.size() - 1) * ratio));
    }


    FUNC_T static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> 
    add_padding_to_feature_image(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& in_mat,
                                int padding_size_x, //填充
                                int padding_size_y) {

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> res_img;
        int size_w = in_mat.cols();
        int size_h = in_mat.rows();

        res_img.resize(size_h + 2 * padding_size_y, size_w + 2 * padding_size_x);
        // res_img.setZero();
        res_img.block(padding_size_y, padding_size_x, size_h, size_w) = in_mat;

        // Padding four corners

        // Top left
        res_img.block(0, 0, padding_size_y, padding_size_x) = in_mat.block(size_h - padding_size_y, size_w - padding_size_x, padding_size_y, padding_size_x);
        // Top right
        res_img.block(0, size_w + padding_size_x, padding_size_y, padding_size_x) = in_mat.block(size_h - padding_size_y, 0, padding_size_y, padding_size_x);
        // Bottom left
        res_img.block(size_h + padding_size_y, 0, padding_size_y, padding_size_x) = in_mat.block(0, size_w - padding_size_x, padding_size_y, padding_size_x);
        // Bottom right
        res_img.block(size_h + padding_size_y, size_w + padding_size_x, padding_size_y, padding_size_x) = in_mat.block(0, 0, padding_size_y, padding_size_x);

        // Padding four blocks
        // Up
        res_img.block(0, padding_size_x, padding_size_y, size_w) = in_mat.block(size_h - padding_size_y, 0, padding_size_y, size_w);
        // Down
        res_img.block(size_h + padding_size_y, padding_size_x, padding_size_y, size_w) = in_mat.block(0, 0, padding_size_y, size_w);
        // Left
        res_img.block(padding_size_y, 0, size_h, padding_size_x) = in_mat.block(0, size_w - padding_size_x, size_h, padding_size_x);
        // Right
        res_img.block(padding_size_y, size_w + padding_size_x, size_h, padding_size_x) = in_mat.block(0, 0, size_h, padding_size_x);

        return res_img;
    }


    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> 
    apply_guassian_blur(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& in_mat,
                        int kernel_size,
                        float sigma) {
        cv::Size kernel = cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1);
        cv::Mat cv_img;
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> res_img;
        res_img = add_padding_to_feature_image(in_mat, kernel_size + 1, kernel_size + 1);
        cv::eigen2cv(res_img, cv_img);
        cv::GaussianBlur(cv_img, cv_img, kernel, sigma);
        cv::cv2eigen(cv_img, res_img);
        return res_img.block(kernel_size + 1, kernel_size + 1, in_mat.rows(), in_mat.cols());
    }


    std::string get_frame_info() {
        char ss_str[10000] = "";
        std::stringstream ss(ss_str);
        ss << "===== Frame info =====" << std::endl;
        ss << "Total cell numbers: " << m_set_cell.size() << std::endl;
        ss << "Num_frames: " << m_accumulate_frames << std::endl;
        ss << "Line_cell_all: " << m_feature_vecs_line.size() << std::endl;
        ss << "Plane_cell_all: " << m_feature_vecs_plane.size() << std::endl;
        ss << "==========" << std::endl;
        return ss.str();
    }

    //called by finished keyframe
    //计算finished keyframe 对应的2D直方图
    FUNC_T void generate_feature_img(std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>>& feature_vecs_line,
                                     std::vector<Eigen::Matrix<DATA_TYPE, 3, 1>>& feature_vecs_plane,
                                     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& feature_img_line,
                                     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& feature_img_plane) {
        Eigen::Matrix<DATA_TYPE, 3, 3> eigen_vector;
        Eigen::Matrix<DATA_TYPE, 3, 1> eigen_val;

        eigen_decompose_of_featurevector(feature_vecs_plane, eigen_vector, eigen_val);
        eigen_vector = eigen_vector.rowwise().reverse().eval(); //特征向量按特征值从大到小排列
        eigen_val = eigen_val.colwise().reverse().eval(); 
        eigen_vector.col(2) = eigen_vector.col(0).cross(eigen_vector.col(1)); //计算R
        //screen_out << "Eigen value = " << eigen_val.transpose() << std::endl;

        feature_img_line.resize(PHI_RESOLUTION, THETA_RESOLUTION);
        feature_img_plane.resize(PHI_RESOLUTION, THETA_RESOLUTION);

        feature_img_line.setZero();
        feature_img_plane.setZero();

        int theta_idx = 0;
        int beta_idx = 0;
        Eigen::Matrix<DATA_TYPE, 3, 1> affined_vector;

        for (size_t i = 0; i < feature_vecs_plane.size(); i++) {
            affined_vector = eigen_vector.transpose() * feature_vecs_plane[i]; 
            feature_direction(affined_vector, theta_idx, beta_idx);
            feature_img_plane(theta_idx, beta_idx) = feature_img_plane(theta_idx, beta_idx) + 1;
        }

        for (size_t i = 0; i < feature_vecs_line.size(); i++) {
            affined_vector = eigen_vector.transpose() * feature_vecs_line[i];
            feature_direction(affined_vector, theta_idx, beta_idx);
            feature_img_line(theta_idx, beta_idx) = feature_img_line(theta_idx, beta_idx) + 1;
        }

        m_ratio_nonzero_line = ratio_of_nonzero_in_img(feature_img_line);
        m_ratio_nonzero_plane = ratio_of_nonzero_in_img(feature_img_plane);
        //计算histogram非零项所占比例

        feature_img_line = apply_guassian_blur(feature_img_line, 4, 4);
        feature_img_plane = apply_guassian_blur(feature_img_plane, 4, 4);
        //高斯模糊化
    }



    //called by finished keyframe
    //计算finished keyframe对应的histograms
    FUNC_T void extract_feature_mapping_new(std::set<Mapping_cell_ptr> cell_vec, //finished keyframe 所有cells集合
                                            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& feature_img_line,
                                            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& feature_img_plane,
                                            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& feature_img_line_roi,
                                            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& feature_img_plane_roi,
                                            int if_recompute = 0) {//if_recompute = 0 
        update_features_of_each_cells(if_recompute); //计算每个cell的feature vector和type
        float ratio = 0.90;
        Eigen_Point keyframe_center; //keyframe 所有cells组成的中心
        m_roi_range = get_ratio_range_of_cell(keyframe_center, ratio); //距离keyframe 中心最远距离的ratio比例是多远
        m_feature_vecs_plane.clear();
        m_feature_vecs_line.clear();
        m_feature_vecs_plane_roi.clear();
        m_feature_vecs_line_roi.clear();

        Map_pt_cell_it it;
        m_feature_vecs_plane.reserve(cell_vec.size()); //保存finished关键帧所有planne_cells的Cd
        m_feature_vecs_line.reserve(cell_vec.size());  //保存finished关键帧所有line_cells的Cd
        m_feature_vecs_plane_roi.reserve(cell_vec.size()); //保存finished关键帧中距离keyframe center 一定距离内，所有planne_cells的Cd
        m_feature_vecs_line_roi.reserve(cell_vec.size());  //保存finished关键帧中距离keyframe center 一定距离内，所有line_cells的Cd

        for (auto it = cell_vec.begin(); it != cell_vec.end(); it++) {
            Mapping_cell_ptr cell = *it;

            auto feature_type = cell->m_feature_type;
            if (feature_type == Feature_type::e_feature_line) {
                m_feature_vecs_line.push_back(cell->m_feature_vector.template cast<DATA_TYPE>());
            }
            if (feature_type == Feature_type::e_feature_plane) {
                m_feature_vecs_plane.push_back(cell->m_feature_vector.template cast<DATA_TYPE>());
            }
            if ((cell->get_center() - keyframe_center).norm() < m_roi_range) {
                if (feature_type == Feature_type::e_feature_line) {
                    m_feature_vecs_line_roi.push_back(cell->m_feature_vector.template cast<DATA_TYPE>());
                }
                if (feature_type == Feature_type::e_feature_plane) {
                    m_feature_vecs_plane_roi.push_back(cell->m_feature_vector.template cast<DATA_TYPE>());
                }
            }
        }

        //screen_out << get_frame_info() << std::endl;
        screen_out << "New keyframe, total cell numbers: " << cell_vec.size();
        screen_out << ", num_frames: " << m_accumulate_frames; //为常数300(finishe keyframe积累了300帧）
        screen_out << ", line_cell_all: " << m_feature_vecs_line.size();
        screen_out << ", plane_cell_all: " << m_feature_vecs_plane.size() << std::endl;

        generate_feature_img(m_feature_vecs_line_roi, m_feature_vecs_plane_roi, feature_img_line_roi, feature_img_plane_roi);
        generate_feature_img(m_feature_vecs_line, m_feature_vecs_plane, feature_img_line, feature_img_plane);       
    }



    //ANCHOR keyframe::analyze
    //called by finished keyframes
    FUNC_T int analyze(int if_recompute = 0) { //if_recompute = 0
        update_features_of_each_cells();
        extract_feature_mapping_new(m_set_cell, m_feature_img_line, m_feature_img_plane, m_feature_img_line_roi, m_feature_img_plane_roi, if_recompute);
        return 0;
    }

    FUNC_T std::string to_json_string(int& avail_cell_size = 0) {
        std::string str;
        str.reserve(m_map_pt_cell.size() * 1e4);
        std::stringstream str_s(str);
        str_s << "[";
        avail_cell_size = 0;
        for (auto it = m_set_cell.begin(); it != m_set_cell.end();) {
            Mapping_cell_ptr cell = *it;

            if (avail_cell_size != 0) {
                str_s << ",";
            }
            str_s << cell->to_json_string();
            avail_cell_size++;

            it++;
            if (it == m_set_cell.end()) {
                break;
            }
        }
        str_s << "]";
        return str_s.str();
    }

    FUNC_T void save_to_file(const std::string& path = std::string("./"), const std::string& file_name = std::string("")) {
        ENABLE_SCREEN_PRINTF;
        std::stringstream str_ss;
        Common_tools::create_dir(path);
        if (file_name.compare("") == 0) {
            str_ss << path << "/" << std::setprecision(3) << "mapping.json";
        } else {
            str_ss << path << "/" << file_name.c_str();
        }
        std::fstream ofs;
        ofs.open(str_ss.str().c_str(), std::ios_base::out);
        screen_out << "Save to " << str_ss.str();
        if (ofs.is_open()) {
            int avail_cell_size = 0;
            ofs << to_json_string(avail_cell_size);
            ofs.close();
            screen_out << " Successful. Number of cell = " << avail_cell_size << std::endl;
        } else {
            screen_out << " Fail !!!" << std::endl;
        }
    }

    template <typename T>
    FUNC_T void eigen_decompose_of_featurevector(std::vector<Eigen::Matrix<T, 3, 1>>& feature_vectors, 
                         Eigen::Matrix<T, 3, 3>& eigen_vector, Eigen::Matrix<T, 3, 1>& eigen_val) {
        Eigen::Matrix<double, 3, 3> mat_cov;
        mat_cov.setIdentity();
        // mat_cov.setZero();
        for (size_t i = 0; i < feature_vectors.size(); i++) {
            mat_cov = mat_cov + (feature_vectors[i] * feature_vectors[i].transpose()).template cast<double>();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver;
        eigensolver.compute(mat_cov);
        eigen_val = eigensolver.eigenvalues().template cast<T>();
        eigen_vector = eigensolver.eigenvectors().template cast<T>();
    }


    FUNC_T static float similiarity_of_two_image(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_a, 
                        const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& img_b) {
        assert(((img_a.rows() == img_b.rows()) && (img_a.cols() == img_b.cols())));

        auto img_sub_mea_a = img_a.array() - img_a.mean();
        auto img_sub_mea_b = img_b.array() - img_b.mean();

        float product = ((img_sub_mea_a).cwiseProduct(img_sub_mea_b)).mean();
        int devide_size = img_a.rows() * img_a.cols() - 1;
        float std_a = (img_sub_mea_a.array().pow(2)).sum() / devide_size;
        float std_b = (img_sub_mea_b.array().pow(2)).sum() / devide_size;
        return sqrt(product * product / std_a / std_b);
    };

    void save_feature_image_to_json(std::string file_name) {
        std::fstream ofs;
        ofs.open(file_name.c_str(), std::ios_base::out);
        if (!ofs.is_open()) {
            screen_out << "Open file " << file_name << " fail!!! please check: " << std::endl;
        }
        rapidjson::Document document;
        rapidjson::StringBuffer sb;

#if IF_JSON_PRETTY_WRITTER
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
#else
        rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
#endif
        writer.StartObject();             // Between StartObject()/EndObject(),
        writer.SetMaxDecimalPlaces(1000); // like set_precision
        writer.Key("Cols");
        writer.Int(PHI_RESOLUTION);
        writer.Key("Rows");
        writer.Int(THETA_RESOLUTION);
        Common_tools::save_mat_to_jason_writter(writer, "Line", m_feature_img_line);
        Common_tools::save_mat_to_jason_writter(writer, "Plane", m_feature_img_plane);
        Common_tools::save_mat_to_jason_writter(writer, "Line_roi", m_feature_img_line_roi);
        Common_tools::save_mat_to_jason_writter(writer, "Plane_roi", m_feature_img_plane_roi);
        writer.EndObject();
        screen_out << "Save feature images to " << file_name << std::endl;
        ofs << std::string(sb.GetString()) << std::endl;
        ofs.close();
    }

    void display() {
        m_if_verbose_screen_printf = 0;
        for (auto it = m_set_cell.begin(); it != m_set_cell.end(); it++) {
            screen_out << "Center of cell is: " << (*(it))->get_center().transpose() << std::endl;
        }
        m_if_verbose_screen_printf = 1;
    }
};
