#ifndef CAMLASERCALIB_H
#define CAMLASERCALIB_H

#include <ros/ros.h>
#include "pcl_util.h"
//! Msg
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include "laser_geometry/laser_geometry.h"
// !PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
//!opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
//!MarkerDetector
#include "MarkerDetector.h"

using namespace std;
using namespace pcl;
using namespace cv;

/*!
 * parameter used to get the rectangle cut area of point cloud
 * laset coordinate:x~left,y~front,z~up
 * camera coordinate:x~left,y~down,z~front
 * so we shoud align the two coordinate to before we get the calibtation matrix
 */
struct Parameter
{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};
namespace camLaserCalib {

class CamLaserCalib
{
public:
    /*!
     * Constructor.
     */
    CamLaserCalib (ros::NodeHandle nodehandle, const string&  imgCloudPointsFile, const string&  calibFile);
    /*!
     * Deconstructor.
     */
    virtual ~CamLaserCalib () {}
    /*!
     * image and laser topic callback function using message filter
     */
    void img_pc2_Callback(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::PointCloud2ConstPtr& pc2);
    /*!
     * dynamic param callback function
     */
    void cfgCallback(const Parameter cfg);
    /*!
     * grab cv::mat from image topic
     */
    void grabImg(const sensor_msgs::ImageConstPtr& img);
    /*!
     * get qrcode center coordinate from image
     */
    void getImgPoint();
    /*!
     * get the calibration param from calib.yml file
     */
    int readCalibPara(string filename);
    /*!
     * publish point cloud message
     */
    void publishCloud();
    //! ROS Subscribe point cloud message
    string strSub_pc2_;
    //! ROS Subscribe image message
    string strSub_img_;

private:

    /*!
     * Read the parameters.
     */
    void readParameters();
    //! dynamic config parameters
    Parameter cfg_;
    //! MarkerDetect
    MarkerDetector markCapture_;
    //! ROS nodehandle
    ros::NodeHandle nodehandle_;
    //! ROS Publisher
    ros::Publisher pub_pc2_cut_;   //cut area of point cloud
    ros::Publisher pub_plane_;     //estimated plane of the cutted point cloud area
    ros::Publisher pub_imgColorPc_;//the color point cloud
    ros::Time pc2Time_;             //pulish message topic time
    //! pcl pointcloud
    Cloud::Ptr pc_;                 //transform pointcloud2(pc2) to pointcloud(pc)
    Cloud::Ptr pc_cut_;             //rectangle cloud
    ColorCloud::Ptr cloudPlane_;    //estamted plane
    ColorCloud::Ptr imgColorCloud_; //color cloud points draw with image color
    PointCT cloudPoint_;            //cloud point center of the board
    //! pcl Parameters
    double distance_threshold_;     //param used to get the point cloud plane
    pcl::ModelCoefficients::Ptr coefficients_;   //pcl plane param
    pcl::PointIndices::Ptr inlierIndices_;       //pcl plane inlier indices
    //!image message
    cv::Mat img_;                   //grab image
    Point2f imgPoint_;              //image point center of the board
    //!image calibration param
    cv::Mat cameraExtrinsicMat_;    //camera laser calibration param
    cv::Mat cameraMat_;             //camera internal parameter
    cv::Mat distCoeff_;             //camera distortion parameter
    cv::Size imageSize_;            //image size
    cv::Mat R_;                     //lasr to image rotation matrix
    cv::Mat T_;                     //lasr to image translation matrix
    bool onlyDrawPointsColor_;      //change in launch file.flag to judge if just draw points color or estimated ceter points of QR code board in both coordinates
    //!read and write param file
    string calibFile_;              //cameta laser calibration file
    ofstream imageCloudPoints_;     //QR code board center points in image and laser coordinate
};

}
#endif // CAMLASERCALIB_H
