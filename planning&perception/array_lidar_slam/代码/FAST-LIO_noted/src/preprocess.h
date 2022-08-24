#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// 枚举类型：表示支持的雷达类型
enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUST64
}; //{1, 2, 3}

// 枚举类型：表示特征点的类型
enum Feature
{
  Nor,        // 正常点
  Poss_Plane, // 可能的平面点
  Real_Plane, // 确定的平面点
  Edge_Jump,  // 有跨越的边
  Edge_Plane, // 边上的平面点
  Wire,       // 线段 这个也许当了无效点？也就是空间中的小线段？
  ZeroPoint   // 无效点 程序中未使用
};
// 枚举类型：位置标识
enum Surround
{
  Prev, // 前一个
  Next  // 后一个
};

// 枚举类型：表示有跨越边的类型
enum E_jump
{
  Nr_nor,  // 正常
  Nr_zero, // 0
  Nr_180,  // 180
  Nr_inf,  // 无穷大 跳变较远？
  Nr_blind // 在盲区？
};
// orgtype类：用于存储激光雷达点的一些其他属性
struct orgtype
{
  double range; // 点云在xy平面离雷达中心的距离
  double dista; // 当前点与后一个点之间的距离
  //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];  // 这个是角OAM和角OAN的cos值
  double intersect; // 这个是角MAN的cos值
  E_jump edj[2];    // 前后两点的类型
  Feature ftype;    // 点类型

  // 构造函数
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor; //默认为正常点
    intersect = 2;
  }
};

// velodyne数据结构
namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    float time;                     // 时间
    uint16_t ring;                  // 点所属的圈数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace velodyne_ros

// 注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))

// ouster数据结构
namespace ouster_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    uint32_t t;                     // 时间
    uint16_t reflectivity;          // 反射率
    uint8_t ring;                   // 点所属的圈数
    uint16_t ambient;               // 没用到
    uint32_t range;                 // 距离
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace ouster_ros

// clang-format off
// 注册ouster的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// Preproscess类：用于对激光雷达点云数据进行预处理
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();  // 构造函数
  ~Preprocess(); // 析构函数
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);  // 对Livox自定义Msg格式的激光雷达数据进行处理
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);     // 对ros的Msg格式的激光雷达数据进行处理
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;  // 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE;  // 雷达类型、采样间隔、扫描线数、扫描频率
  double blind;  // 最小距离阈值(盲区)
  bool feature_enabled, given_offset_time;  // 是否提取特征、是否进行时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn;  // 发布全部点、发布平面点、发布边缘点
    

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);  // 用于对Livox激光雷达数据进行处理
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);   // 用于对ouster激光雷达数据进行处理
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg); // 用于对velodyne激光雷达数据进行处理
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);  // 没有用到
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
