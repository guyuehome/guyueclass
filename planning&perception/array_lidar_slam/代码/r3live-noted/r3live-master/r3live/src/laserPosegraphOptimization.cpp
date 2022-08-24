#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include "csignal"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <ad_localization_msgs/NavStateInfo.h>

#include "scancontext/tic_toc.h"
//TODO: 
#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero
Pose6D currRTK {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
tf::Transform odom_to_enu; // init rtk pose in enu
tf::Transform T_livox2gnss;

std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::deque<ad_localization_msgs::NavStateInfo::ConstPtr> rtkBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;
std::mutex rtkBufLock;
std::mutex rtkKFLock;
double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::unordered_map<int, Pose6D> keyframePosesRTK;
std::vector<double> keyframeTimes;
std::vector<ros::Time> keyframeTimesOri;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Diagonal::shared_ptr RTKNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

bool useRTK = false;
sensor_msgs::NavSatFix::ConstPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
bool hasRTKforThisKF = false;
bool rtkOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO, pubOdomResultForInteractiveSlam, pubMapResultForInteractiveSlam;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;
ros::Publisher pubOdomRepubVerifier;

// save TUM format path
std::string save_path;
std::string save_pcd_path;
std::shared_ptr<std::ofstream> odo;

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

tf::Transform Pose6DtoTransform(const Pose6D& p)
{
    tf::Vector3 t(p.x, p.y, p.z);
    tf::Quaternion q;
    q.setRPY(p.roll, p.pitch, p.yaw);
    tf::Transform tran(q, t);
    return tf::Transform(q, t);
} // Pose6DtoTransform
Pose6D TransformtoPose6D(const tf::Transform& trans)
{
    double roll, pitch, yaw;
    tf::Matrix3x3(trans.getRotation()).getRPY(roll, pitch, yaw);
    tf::Vector3 t=trans.getOrigin();
    return Pose6D{t[0], t[1], t[2], roll, pitch, yaw}; 
} // TransformtoPose6D
Pose6D PoseInterpolation(const Pose6D& p_before, const Pose6D& p_after, double step_length) {
    tf::Quaternion q_before, q_after;
    q_before.setRPY(p_before.roll, p_before.pitch, p_before.yaw);
    q_after.setRPY(p_after.roll, p_after.pitch, p_after.yaw);
    tf::Vector3 t_before(p_before.x, p_before.y, p_before.z);
    tf::Vector3 t_after(p_after.x, p_after.y, p_after.z);
    tf::Quaternion q_curr = q_before.slerp(q_after, step_length);
    tf::Vector3 t_curr = t_before.lerp(t_after, step_length);
    return TransformtoPose6D(tf::Transform(q_curr, t_curr));
} // PoseInterpolation

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
  static double last_odom_time = 0;
	mBuf.lock();
    if (!useRTK && keyframeTimes.size() &&
        abs(_laserOdometry->header.stamp.toSec() - last_odom_time) > 1) {
        std::cout << std::endl << std::endl << std::endl << std::endl;
        std::cout << "the time gap is " << _laserOdometry->header.stamp.toSec() - keyframeTimes.back() << std::endl;
        std::cout << "receive the next rosbag" << std::endl;
        std::cout << std::endl << std::endl << std::endl << std::endl;
        gtSAMgraphMade = false;
    }
	odometryBuf.push(_laserOdometry);
  last_odom_time = _laserOdometry->header.stamp.toSec();
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void rtkHandler(const ad_localization_msgs::NavStateInfo::ConstPtr &_rtk)
{
    static double last_rtk_time = 0;
    rtkBufLock.lock();
    if (rtkBuf.size() &&
        abs(_rtk->header.stamp.toSec() - last_rtk_time) > 0.2) {
        std::cout << std::endl << std::endl << std::endl << std::endl;
        std::cout << "the time gap is " << _rtk->header.stamp.toSec() - last_rtk_time << std::endl;
        std::cout << "receive the next rosbag" << std::endl;
        std::cout << std::endl << std::endl << std::endl << std::endl;
        rtkBuf.clear();
        rtkOffsetInitialized = false;
        gtSAMgraphMade = false;
    }
    rtkBuf.push_back(_rtk);
    last_rtk_time = _rtk->header.stamp.toSec();
    rtkBufLock.unlock();
} // gpsHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    gtsam::Vector RTKNoiseVector6(6);
    RTKNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    RTKNoise = noiseModel::Diagonal::Variances(RTKNoiseVector6);

    double loopNoiseScore = 0.001; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

Pose6D convertOdomToENU(const Pose6D &pose_curr)
{
    tf::Transform T_curr = Pose6DtoTransform(pose_curr);
    tf::Transform T_curr_enu;
    T_curr_enu.mult(odom_to_enu, T_curr);
    return TransformtoPose6D(T_curr_enu);
} // getOdom

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}
void pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
    int counter = 0;

    mKF.lock(); 
    laserCloudMapPGO->clear();
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "/camera_init";
    pubMapAftPGO.publish(laserCloudMapPGOMsg);
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "/camera_init";
    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "/camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "/camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock(); 
    pubOdomAftPGO.publish(odomAftPGO); // last pose 
    pubPathAftPGO.publish(pathAftPGO); // poses 

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, "/camera_init", "/aft_pgo"));
} // pubPath

void updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, pcl::PointCloud<PointType>::Ptr& nearKeyframesViz, const int& key, const int& submap_size)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    mKF.lock();
    tf::Transform T_key = Pose6DtoTransform(keyframePosesUpdated[key]);
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        tf::Transform T_keyNear = Pose6DtoTransform(keyframePosesUpdated[keyNear]);
        Pose6D pose_relative = TransformtoPose6D(T_key.inverseTimes(T_keyNear));
        *nearKeyframes += *local2global(keyframeLaserClouds[keyNear], pose_relative);
        *nearKeyframesViz += *local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        
    }
    mKF.unlock(); 

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
    downSizeFilterICP.setInputCloud(nearKeyframesViz);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframesViz = *cloud_temp;
} // loopFindNearKeyframesCloud


std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // Check that the loopGap is correct
    Pose6D pose_loop;
    Pose6D pose_curr;
    if (useRTK) {
      rtkKFLock.lock();
      auto it_loop = keyframePosesRTK.find(_loop_kf_idx);
      auto it_curr = keyframePosesRTK.find(_curr_kf_idx);
      if (it_loop != keyframePosesRTK.end() &&
          it_curr != keyframePosesRTK.end()) {
        pose_loop = it_loop->second;
        pose_curr = it_curr->second;
        rtkKFLock.unlock();
      } else {
        rtkKFLock.unlock();
        std::cout << "this loop don't have rtk init pose, so reject" << std::endl;
        return std::nullopt;
      }
    } else {
      pose_loop = keyframePosesUpdated[_loop_kf_idx];
      pose_curr = keyframePosesUpdated[_curr_kf_idx];
    }
    double loopGapThreshold = 5.0;
    double dx = pose_loop.x - pose_curr.x, dy = pose_loop.y - pose_curr.y;
    double loopGap = std::sqrt(dx * dx + dy * dy);
    if (loopGap > loopGapThreshold) {
      std::cout << "loopGap is too big " << std::endl;
      std::cout << "loopGap = " << loopGap << std::endl;
      std::cout << "pose_curr = " << pose_curr.x << " " << pose_curr.y << " " << pose_curr.z << std::endl;
      std::cout << "pose_loop = " << pose_loop.x << " " << pose_loop.y << " " << pose_loop.z << std::endl;
      return std::nullopt;
    }

    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr currKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr currKeyframeCloudViz(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr icpKeyframeCloudViz(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloudViz(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(currKeyframeCloud, currKeyframeCloudViz, _curr_kf_idx, 0); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, targetKeyframeCloudViz, _loop_kf_idx, historyKeyframeSearchNum); 

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // cal the the initial position transform
    Eigen::Isometry3d T_init = Eigen::Isometry3d::Identity();
    tf::Transform T_relative;
    tf::Transform T_loop = Pose6DtoTransform(pose_loop);
    tf::Transform T_curr = Pose6DtoTransform(pose_curr);
    T_relative = T_loop.inverseTimes(T_curr);
    tf::transformTFToEigen (T_relative, T_init);

    // Align pointclouds
    icp.setInputSource(currKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result, T_init.matrix().cast<float>());

    float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe. 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }
    // icp verification 
    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloudViz, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "/camera_init";
    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);

    *icpKeyframeCloudViz += *local2global(unused_result, pose_loop);
    sensor_msgs::PointCloud2 icpKeyframeCloudMsg;
    pcl::toROSMsg(*icpKeyframeCloudViz, icpKeyframeCloudMsg);
    icpKeyframeCloudMsg.header.frame_id = "/camera_init";
    pubLoopScanLocal.publish(icpKeyframeCloudMsg);

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while(1)
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
            timeLaser = fullResBuf.front()->header.stamp.toSec();
            if (abs(timeLaserOdometry - timeLaser) > 0.01) {
              std::cout << "timeLaserOdometry - timeLaser = " << timeLaserOdometry - timeLaser << std::endl;
              return;
            }
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            ros::Time time_stamp = odometryBuf.front()->header.stamp;
            odometryBuf.pop();
            mBuf.unlock(); 

            // check the rtk data
            if (useRTK) {
              rtkBufLock.lock();
              if (rtkBuf.empty() || 
                  rtkBuf.back()->header.stamp.toSec() < timeLaserOdometry ||
                  rtkBuf.front()->header.stamp.toSec() > timeLaserOdometry) {
                  std::cout << "rtk data is abnormal" << std::endl;
                  std::cout << "rtk's front time is " << rtkBuf.front()->header.stamp << std::endl;
                  std::cout << "odo's front time is " << time_stamp << std::endl;
                  std::cout << "rtk's back time is " << rtkBuf.back()->header.stamp << std::endl;
                  std::cout << "rtk's size is " << rtkBuf.size() << std::endl;
                  rtkBufLock.unlock();
                  return;
              }
              // find the first and nearest rtk
              ad_localization_msgs::NavStateInfo::ConstPtr preRTK;
              while (!rtkBuf.empty() && rtkBuf.front()->header.stamp.toSec() < timeLaserOdometry) {
                preRTK = rtkBuf.front();
                rtkBuf.pop_front();
              }
              rtkBufLock.unlock();
              double frame_time_interval = rtkBuf.front()->header.stamp.toSec() - preRTK->header.stamp.toSec();
              if (preRTK->header.stamp.toSec() < timeLaserOdometry &&
                  rtkBuf.front()->header.stamp.toSec() > timeLaserOdometry &&
                  abs(frame_time_interval) < 0.1) {
                    Pose6D rtkPoseBefore {preRTK->position_enu.x, preRTK->position_enu.y, preRTK->position_enu.z, 
                                          preRTK->roll, preRTK->pitch, preRTK->yaw};
                    Pose6D rtkPoseAfter {rtkBuf.front()->position_enu.x, rtkBuf.front()->position_enu.y, rtkBuf.front()->position_enu.z, 
                                          rtkBuf.front()->roll, rtkBuf.front()->pitch, rtkBuf.front()->yaw};
                    double step_length = (timeLaserOdometry - preRTK->header.stamp.toSec()) / frame_time_interval;
                    currRTK = PoseInterpolation(rtkPoseBefore, rtkPoseAfter, step_length);
                    currRTK = TransformtoPose6D(Pose6DtoTransform(currRTK) * T_livox2gnss);
                    hasRTKforThisKF = true;
              } else {
                    hasRTKforThisKF = false;
                    std::cout << "rtk data's gap is too long" << std::endl;
                    return;
              }
            }
            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue;

            if (useRTK && !rtkOffsetInitialized) {
              odom_to_enu = Pose6DtoTransform(currRTK) * Pose6DtoTransform(pose_curr).inverse();
              Pose6D odom_to_enu_pose = TransformtoPose6D(odom_to_enu);
              odom_to_enu_pose.roll = 0;
              odom_to_enu_pose.pitch = 0;
              odom_to_enu = Pose6DtoTransform(odom_to_enu_pose);
              std::cout << "enu pose is " << currRTK.x << " " << currRTK.y << " " << currRTK.z << " "
                        << currRTK.roll << " " << currRTK.pitch << " " << currRTK.yaw << " "
                        << std::endl;
              rtkOffsetInitialized = true;
            }
            // convert the current pose to enu
            if (useRTK) {
              if (rtkOffsetInitialized) {
                pose_curr = convertOdomToENU(pose_curr);
              } else {
                std::cout << "rtk's Initialize is abnormal" << time_stamp << std::endl;
                break;
              }
            }
            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock(); 
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);
            keyframeTimesOri.push_back(time_stamp);
            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);
            laserCloudMapPGORedraw = true;
            mKF.unlock(); 

            rtkKFLock.lock();
            if (hasRTKforThisKF) {
              keyframePosesRTK[keyframePoses.size() - 1] = currRTK;
            }
            rtkKFLock.unlock();
            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */) {
                int init_node_idx = 0;
                if (curr_node_idx > 1) {
                    gtsam::Vector priorNoiseVector6(6);
                    // set a big priorNoise
                    priorNoiseVector6 << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;
                    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);
                    init_node_idx = curr_node_idx;
                }
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    // runISAM2opt(); 
                }
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // // rtk factor 
                    // if(hasRTKforThisKF) {
                    //     gtsam::Pose3 poseRTK = Pose6DtoGTSAMPose3(currRTK);
                    //     gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(curr_node_idx, poseRTK, RTKNoise));
                    //     cout << "RTK factor added at node " << curr_node_idx << endl;
                    // }
                    initialEstimate.insert(curr_node_idx, poseTo);                
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performSCLoopClosure(void)
{
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change 
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void process_icp(void)
{
    while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // runISAM2opt();
                mtxPosegraph.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = 10.0; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = 1; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            cout << "running isam2 optimization ..." << endl;
            mtxPosegraph.unlock();
        }
    }
}

void process_viz_map(void)
{
    float vizmapFrequency = 0.5; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        // if(recentIdxUpdated > 1) {
        pubMap();
        // }
    }
} // pointcloud_viz

void process_interactive_slam_result() {
  ros::Rate rate(50);
  mKF.lock();
  for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    // pub Map for interactive slam
    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*keyframeLaserClouds[node_idx], laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "/camera_init";
    laserCloudMapPGOMsg.header.stamp = keyframeTimesOri.at(node_idx);
    pubMapResultForInteractiveSlam.publish(laserCloudMapPGOMsg);
    // pub Odom for interactive slam
    const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
    nav_msgs::Odometry odomAftPGOthis;
    odomAftPGOthis.header.frame_id = "/camera_init";
    odomAftPGOthis.child_frame_id = "/aft_pgo";
    odomAftPGOthis.header.stamp = laserCloudMapPGOMsg.header.stamp;
    odomAftPGOthis.pose.pose.position.x = pose_est.x;
    odomAftPGOthis.pose.pose.position.y = pose_est.y;
    odomAftPGOthis.pose.pose.position.z = pose_est.z;
    odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
    pubOdomResultForInteractiveSlam.publish(odomAftPGOthis); // last pose 
    rate.sleep();
  }
  mKF.unlock();
} // pointcloud_viz

void save_odo() {
  std::cout << "save_path = " << save_path << std::endl;
  mKF.lock();
  for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
    const Pose6D& pose_rtk = TransformtoPose6D(Pose6DtoTransform(pose_est) * T_livox2gnss.inverse());
    std::stringstream ss_x, ss_y, ss_z;
    ss_x.precision(7);
    ss_x.setf(std::ios::fixed);
    ss_y.precision(7);
    ss_y.setf(std::ios::fixed);
    ss_z.precision(7);
    ss_z.setf(std::ios::fixed);
    ss_x << pose_rtk.x;
    ss_y << pose_rtk.y;
    ss_z << pose_rtk.z;
    *odo //<< std::setw(20) 
        // << node_idx << " "
        << keyframeTimesOri.at(node_idx) << " " 
        << ss_x.str() << " " << ss_y.str() << " " << ss_z.str() << " "
        << pose_est.roll << " " << pose_est.pitch << " " << pose_est.yaw << " "
        // << 0 << " " << 0 << " " << 0 << " " << 1
        << std::endl;
  }
  mKF.unlock();
} // pointcloud_viz

void save_map() {
  std::string all_points_dir(save_pcd_path + "scans_lc.pcd");
  std::cout << "all_points_dir = " << all_points_dir << std::endl;
  mKF.lock();
  laserCloudMapPGO->clear();
  for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
  }
  pcl::io::savePCDFile(all_points_dir, *laserCloudMapPGO);
  mKF.unlock();
} // pointcloud_viz

void MySigintHandler(int sig)
{
  // Before exit the main thread, the odom result and the map data will be saved and sent to interactive_slam
	ROS_INFO("shutting down!");
  save_odo();
  save_map();
  process_interactive_slam_result();
	ros::shutdown();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserPGO");
	ros::NodeHandle nh;

  nh.param<std::string>("save_path", save_path, "/");
  std::cout << "save_pcd_path = " << save_path << std::endl;
  nh.param<std::string>("save_pcd_path", save_pcd_path, "/");
  std::cout << "save_pcd_path = " << save_pcd_path << std::endl;
  odo.reset(new std::ofstream(save_path));

	nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 2.0); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", keyframeDegGap, 10.0); // pose assignment every k deg rot 
  keyframeRadGap = deg2rad(keyframeDegGap);
  std:: cout << "keyframeMeterGap = " << keyframeMeterGap << std:: endl;
  std:: cout << "keyframeDegGap = " << keyframeDegGap << std:: endl;
  std:: cout << "keyframeRadGap = " << keyframeRadGap << std:: endl;
	nh.param<double>("sc_dist_thres", scDistThres, 0.2);
	nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor 

  nh.param<bool>("useRTK", useRTK, false);
  std:: cout << "useRTK = " << useRTK << std:: endl;

  std::vector<double> vecTlivox2gnss;
	ros::param::get("Extrinsic_T_livox2gnss", vecTlivox2gnss);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);
  initNoises();

  scManager.setSCdistThres(scDistThres);
  scManager.setMaximumRadius(scMaximumRadius);

  float filter_size = 0.4; 
  downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
  downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

  double mapVizFilterSize;
	nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames 
  downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
  ros::Subscriber subRTK = nh.subscribe<ad_localization_msgs::NavStateInfo>("/localization/navstate_info", 100, rtkHandler);

	pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);
	pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);
	pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);

  // save interactive_slam_pose
  pubOdomResultForInteractiveSlam = nh.advertise<nav_msgs::Odometry>("/interactive_slam_odom", 1000);
  pubMapResultForInteractiveSlam = nh.advertise<sensor_msgs::PointCloud2>("/interactive_slam_map", 1000);
  signal(SIGINT, MySigintHandler);
  Eigen::Isometry3d T_livox2gnss_temp;
  T_livox2gnss_temp.matrix() << vecTlivox2gnss[0], vecTlivox2gnss[1], vecTlivox2gnss[2], vecTlivox2gnss[3],
	                              vecTlivox2gnss[4], vecTlivox2gnss[5], vecTlivox2gnss[6], vecTlivox2gnss[7],
	                              vecTlivox2gnss[8], vecTlivox2gnss[9], vecTlivox2gnss[10], vecTlivox2gnss[11],
                                vecTlivox2gnss[12], vecTlivox2gnss[13], vecTlivox2gnss[14], vecTlivox2gnss[15];
  tf::transformEigenToTF(T_livox2gnss_temp, T_livox2gnss);
	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection 
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	std::thread viz_path {process_viz_path}; // visualization - path (high frequency)
 	ros::spin();
  
	return 0;
}