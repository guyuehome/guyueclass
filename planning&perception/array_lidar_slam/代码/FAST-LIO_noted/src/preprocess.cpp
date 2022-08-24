#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;      // 有效点集合,大于10m则是盲区
  N_SCANS = 6;         //多线激光雷达的线数
  group_size = 8;      // 8个点为一组
  disA = 0.01;         // 点集合的距离阈值,判断是否为平面
  disB = 0.1;          // 点集合的距离阈值,判断是否为平面
  p2l_ratio = 225;     // 点到线的距离阈值，需要大于这个值才能判断组成面
  limit_maxmid = 6.25; // 中点到左侧的距离变化率范围
  limit_midmin = 6.25; // 中点到右侧的距离变化率范围
  limit_maxmin = 3.24; // 左侧到右侧的距离变化率范围
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;   //点与点距离超过两倍则认为遮挡
  edgeb = 0.1; //点与点距离超过0.1m则认为遮挡
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;        //三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
  given_offset_time = false; //是否提供时间偏移

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);       //角度大于170度的点跳过，认为在
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);   //角度小于8度的点跳过
  cos160 = cos(cos160 / 180 * M_PI);                     //夹角限制
  smallp_intersect = cos(smallp_intersect / 180 * M_PI); //三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;    //是否提取特征点
  lidar_type = lid_type;        //雷达的种类
  blind = bld;                  //最小距离阈值，即过滤掉0～blind范围内的点云
  point_filter_num = pfilt_num; //采样间隔，即每隔point_filter_num个点取1个点
}
/**
 * @brief Livox激光雷达点云预处理函数
 *
 * @param msg livox激光雷达点云数据，格式为livox_ros_driver::CustomMsg
 * @param pcl_out 输出处理后的点云数据，格式为pcl::PointCloud<pcl::PointXYZINormal>
 */
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}
/**
 * @brief 对Livox激光雷达点云数据进行预处理
 *
 * @param msg livox激光雷达点云数据，格式为livox_ros_driver::CustomMsg
 */
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  // 清除之前的点云缓存
  pl_surf.clear();             // 清除之前的平面点云缓存
  pl_corn.clear();             // 清除之前的角点云缓存
  pl_full.clear();             // 清除之前的全点云缓存
  double t1 = omp_get_wtime(); // 后面没用到
  int plsize = msg->point_num; // 一帧中的点云总个数
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize); // 分配空间
  pl_surf.reserve(plsize); // 分配空间
  pl_full.resize(plsize);  // 分配空间

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize); // 每一个scan保存的点云数量
  }
  uint valid_num = 0; // 有效的点云数

  // 特征提取（FastLIO2默认不进行特征提取）
  if (feature_enabled)
  {
    // 分别对每个点云进行处理
    for (uint i = 1; i < plsize; i++)
    {
      // 只取线数在0~N_SCANS内并且回波次序为0或者1的点云
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;                                    // 点云x轴坐标
        pl_full[i].y = msg->points[i].y;                                    // 点云y轴坐标
        pl_full[i].z = msg->points[i].z;                                    // 点云z轴坐标
        pl_full[i].intensity = msg->points[i].reflectivity;                 // 点云强度
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 使用曲率作为每个激光点的时间

        bool is_new = false;
        // 只有当当前点和上一点的间距足够大（>1e-7），才将当前点认为是有用的点，分别加入到对应line的pl_buff队列中
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]); // 将当前点加入到对应line的pl_buff队列中
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();
    // 对每个line中的激光雷达分别进行处理
    for (int j = 0; j < N_SCANS; j++)
    {
      // 如果该line中的点云过小，则继续处理下一条line
      if (pl_buff[j].size() <= 5)
        continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y; // 计算每个点到机器人本身的距离
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz; // 计算两个间隔点的距离
      }
      //因为i最后一个点没有i+1了所以就单独求了一个range，没有distance
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;
      give_feature(pl, types); //给特征
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    // 分别对每个点云进行处理
    for (uint i = 1; i < plsize; i++)
    {
      // 只取线数在0~N_SCANS内并且回波次序为0或者1的点云
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num++; // 有效的点云数

        // 等间隔降采样
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;                                    // 点云x轴坐标
          pl_full[i].y = msg->points[i].y;                                    // 点云y轴坐标
          pl_full[i].z = msg->points[i].z;                                    // 点云z轴坐标
          pl_full[i].intensity = msg->points[i].reflectivity;                 // 点云强度
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

          // 只有当当前点和上一点的间距足够大（>1e-7），并且在最小距离阈值之外，才将当前点认为是有用的点，加入到pl_surf队列中
          if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y > blind))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind)
        continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      //这里好像没有用 因为atan2()返回在-pi到pi之间
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      if (pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < blind)
        continue;

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0; // Normal结构体表示给定点所在样本曲面上的法线方向,以及对应曲率的测量值
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  bool is_first[MAX_LINE_NUM];
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  double omega_l = 3.61;                 // scan angular velocity
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].time > 0) //假如提供了每个点的时间戳
  {
    given_offset_time = true; //提供时间偏移
  }
  else //没有提供每个点的时间戳
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578; //寻找终止yaw角
        break;
      }
    }
  }

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS)
        continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

      if (!given_offset_time) //假如没有提供时间
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer]) //给每条线第一个赋予yaw角 时间归0
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;           //对每一条线最后一个点的时间赋值
          time_last[layer] = added_pt.curvature; //对每一条线最后一个点的时间赋值
          continue;
        }
        //对每个点计算时间
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }
        //新得到的点的时间不能早于之前的最后一个点的时间
        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2)
        continue; //点太少跳过
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        // added_pt.curvature = pl_orig.points[i].t;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time: %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature, prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time, pl_orig.points[i].t);
        }
      }
    }
  }

  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_surf, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}
/**
 * @brief 对于每条line的点云提取特征
 *
 * @param pl  pcl格式的点云 输入进来一条扫描线上的点
 * @param types  点云的其他属性
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size(); //单条线的点数
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
  //不能在盲区 从这条线非盲区的点开始
  while (types[head].range < blind)
  {
    head++;
  }

  // Surf
  // group_size默认等于8
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0; //判断当前点后面是否还有8个点 够的话就逐渐减少

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero()); //当前平面的法向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero()); //上一个平面的法向量

  uint i_nex = 0, i2;  // i2为当前点的下一个点
  uint last_i = 0;     // last_i为上一个点的保存的索引
  uint last_i_nex = 0; // last_i_nex为上一个点的下一个点的索引
  int last_state = 0;  //为1代表上个状态为平面 否则为0

  //判断面点
  int plane_type;

  // 拿到8个点用于判断平面
  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind) // 在盲区范围内的点不做处理
      continue;
    {
      continue;
    }

    i2 = i; //更新i2

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct); //求得平面，并返回类型0 1 2

    if (plane_type == 1) //返回1一般默认是平面
    {
      //设置确定的平面点和可能的平面点
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex)
        {
          //把起始点和终止点之间的所有点设置为确定的平面点
          types[j].ftype = Real_Plane;
        }
        else
        {
          //把起始点和终止点设置为可能的平面点
          types[j].ftype = Poss_Plane;
        }
      }
      // if(last_state==1 && fabs(last_direct.sum())>0.5)

      //最开始last_state=0直接跳过
      //之后last_state=1
      //如果之前状态是平面则判断当前点是处于两平面边缘的点还是较为平坦的平面的点
      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707)
        {
          //修改ftype，两个面法向量夹角在45度和135度之间 认为是两平面边缘上的点
          types[i].ftype = Edge_Plane;
        }
        else
        {
          //否则认为是真正的平面点
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      // plane_type=0或2的时候
      i = i_nex;
      last_state = 0; //设置为不是平面点
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;               //更新last_i
    last_i_nex = i_nex;        //更新last_i_nex
    last_direct = curr_direct; //更新last_direct
  }

  //判断边缘点
  plsize2 = plsize > 3 ? plsize - 3 : 0; //如果剩下的点数小于3则不判断边缘点，否则计算哪些点是边缘点
  for (uint i = head + 3; i < plsize2; i++)
  {
    //点不能在盲区 或者 点必须属于正常点和可能的平面点
    if (types[i].range < blind || types[i].ftype >= Real_Plane)
    {
      continue;
    }
    //该点与前后点的距离不能挨的太近
    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z); //当前点组成的向量
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }

      //若当前的前/后一个点在盲区内（4m)
      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound) //若其大于10m
        {
          types[i].edj[j] = Nr_inf; //赋予该点Nr_inf(跳变较远)
        }
        else
        {
          types[i].edj[j] = Nr_blind; //赋予该点Nr_blind(在盲区)
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a; //前/后点指向当前点的向量
      //若雷达坐标系原点为O 当前点为A 前/后一点为M和N
      //则下面OA点乘MA/（|OA|*|MA|）
      //得到的是cos角OAM的大小

      types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm(); //角MAN的cos值

      //前一个点是正常点 && 下一个点在激光线上 && 当前点与后一个点的距离大于0.0225m && 当前点与后一个点的距离大于当前点与前一个点距离的四倍
      //这种边缘点像是7字形这种的边缘？
      if (types[i].angle[j] < jump_up_limit) // cos(170)
      {
        types[i].edj[j] = Nr_180; // M在OA延长线上
      }
      else if (types[i].angle[j] > jump_down_limit) // cos(8)
      {
        types[i].edj[j] = Nr_zero; // M在OA上
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160) //角MAN要小于160度 不然就平行于激光了
      {
        if (edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    //与上面类似
    //前一个点在激光束上 && 后一个点正常 && 前一个点与当前点的距离大于0.0225m && 前一个点与当前点的距离大于当前点与后一个点距离的四倍
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    //前面的是正常点 && (当前点到中心距离>10m并且后点在盲区)
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    //(当前点到中心距离>10m并且前点在盲区) && 后面的是正常点
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    //前后点都不是正常点
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire; //程序中应该没使用 当成空间中的小线段或者无用点了
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  //继续找平面点
  for (uint i = head + 1; i < plsize2; i++)
  {
    //前面、当前、之后三个点都需要不在盲区内
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }
    //前面和当前 当前和之后的点与点之间距离都不能太近
    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }
    //还剩下来一些正常点继续找平面点

    if (types[i].ftype == Nor)
    {
      //求点与点间距的比例 大间距/小间距
      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      //如果夹角大于172.5度 && 间距比例<1.2
      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        //前后三个点认为是平面点
        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }
  //存储平面点
  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    //可能的平面点和确定的平面点
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }
      //通常连着好几个都是面点
      //必须在采样间隔上的平面点才使用（这里的是无差别滤波 从每次新找到面点开始每几个点才取一个）
      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      //跳变较大的边缘边的点 位于平面边缘的点
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      //假如上次找到的面点被无差别滤波掉了，而此时已经到了边缘
      if (last_surface != -1)
      {
        PointType ap;
        //取上次面点到此次边缘线之间的所有点的重心当作一个面点存储进去
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

// 发布topic指令，该函数暂时没用到
void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

//平面判断
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB; // 0.01*sqrt(x^2+y^2)+0.1 基本上可以近似看成是0.1 100m的时候才到0.2
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr; //前后点距离数组
  disarr.reserve(20);

  //距离小 点与点之间较近 先取够8个点
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind)
    {
      curr_direct.setZero(); //距离雷达原点太小了将法向量设置为零向量
      return 2;
    }
    disarr.push_back(types[i_nex].dista); //存储当前点与后一个点的距离
  }

  //看看后续的点有没有满足条件的
  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) //索引超出所有点的个数直接BREAK
      break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero(); //距离雷达原点太小了将法向量设置为零向量
      return 2;
    }
    //最后的i_nex点到i_cur点的距离
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis) //距离i_cur点太远了就直接break
    {
      break;
    }
    disarr.push_back(types[i_nex].dista); //存储当前点与后一个点的距离
    i_nex++;                              // i_nex点加一
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;
    //假设i_cur点为A j点为B i_nex点为C
    //向量AB
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;
    //向量AB叉乘向量AC
    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];
    //物理意义是组成的ABC组成的平行四边形面积的平方(为|AC|*h，其中为B到线AC的距离)

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid)
    {
      leng_wid = lw; //寻找最大面积的平方(也就是寻找距离AC最远的B)
    }
  }

  //|AC|*|AC|/(|AC|*|AC|*h*h)<225
  //也就是h>1/15 B点到AC的距离要大于0.06667m
  //太近了不好拟合一个平面
  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero(); //太近了法向量直接设置为0
    return 0;
  }

  //把两点之间的距离 按从大到小排个顺序
  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }
  //这里可能最近的点还是太近了
  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  //目前还不太懂为什么给AVIA单独弄了一种，其实我觉得全都可以采用上面这种判定方式（虽然FAST-LIO默认不开特征提取）
  if (lidar_type == AVIA)
  {
    //点与点之间距离变化太大的时候 可能与激光束是平行的 就也舍弃了
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize(); //法向量归一化
  return 1;
}
//边缘判断
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind) //前两个点不能在盲区
    {
      return false;
    }
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind) //后两个点不能在盲区
    {
      return false;
    }
  }
  //下面分别对i-2 i-1和i i+1两种情况时点与点间距进行了判断
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;
  //将大小间距进行调换 大在前 小在后
  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {
    //假如间距太大 可能是被遮挡，就不把它当作边缘点
    return false;
  }

  return true;
}
