#include "roi_clip.h"


/**
 * @brief RoiClip 构造函数
 *
 * 使用给定的 ROS 节点句柄初始化 RoiClip 对象。
 *
 * @param node_handle ROS 节点句柄
 * @param private_node_handle ROS 私有节点句柄
 */
RoiClip::RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{  
    private_node_handle.param("roi_x_min", roi_x_min_, 0.0);
    private_node_handle.param("roi_x_max", roi_x_max_, 100.0);
    private_node_handle.param("roi_y_min", roi_y_min_, -20.0);
    private_node_handle.param("roi_y_max", roi_y_max_, 20.0);
    private_node_handle.param("roi_z_min", roi_z_min_, -1.75);
    private_node_handle.param("roi_z_max", roi_z_max_, 2.0);
    // 转换到车辆坐标系下后，将车身点云切除，车辆坐标系中心为后轴中心０点
    // 坐标系参数不确定，暂时不转换
    private_node_handle.param("vehicle_x_min", vehicle_x_min_, -1.2);
    private_node_handle.param("vehicle_x_max", vehicle_x_max_, 3.0);
    private_node_handle.param("vehicle_y_min", vehicle_y_min_, -1.0);
    private_node_handle.param("vehicle_y_max", vehicle_y_max_, 1.0);
    private_node_handle.param("vehicle_z_min", vehicle_z_min_, -1.7);
    private_node_handle.param("vehicle_z_max", vehicle_z_max_, 0.2);
}



/**
 * @brief 获取感兴趣区域（ROI）的点云数据
 *
 * 从输入的点云数据中裁剪出感兴趣区域（ROI）的点云数据，并返回裁剪后的点云数据。
 *
 * @param in 输入的点云数据
 * @param out 裁剪后的点云数据
 *
 * @return 裁剪后的点云数据指针
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,pcl::PointCloud<pcl::PointXYZI>::Ptr &out)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipCloud = ClipVehicle(in);
    if (clipCloud->points.size() > 0)
    {
        for (auto &p : clipCloud->points)
        {
            if (IsIn(p.x, roi_x_min_, roi_x_max_) && IsIn(p.y, roi_y_min_, roi_y_max_) && IsIn(p.z, roi_z_min_, roi_z_max_))
            out->push_back(p);
        }
        ROS_INFO("GetROI sucess");
        return out;

    }

    ROS_ERROR("GetROI fail");
    return nullptr;
}


/**
 * @brief 裁剪车辆点云
 *
 * 根据给定的车辆点云范围，裁剪输入的点云，返回裁剪后的点云。
 *
 * @param in 输入的点云指针
 *
 * @return 裁剪后的点云指针
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::ClipVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in){
    if(in->points.size() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto &p : in->points)
        {
            if (IsIn(p.x, vehicle_x_min_, vehicle_x_max_) && IsIn(p.y, vehicle_y_min_, vehicle_y_max_) && IsIn(p.z, vehicle_z_min_, vehicle_z_max_)){
                
            }
            else out->push_back(p);
        }
        return out;
    }
    return nullptr; 
}

