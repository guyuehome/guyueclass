#include "camLaserCalib.h"
using namespace camLaserCalib;
using namespace std;
using namespace cv;
#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

CamLaserCalib::CamLaserCalib(ros::NodeHandle nodehandle, const string&  imgCloudPointsFile, const string&  calibFile)
    : nodehandle_(nodehandle),
      pc_(new Cloud),
      pc_cut_(new Cloud),
      cloudPlane_(new ColorCloud),
      imgColorCloud_(new ColorCloud),
      coefficients_(new ModelCoefficients),
      inlierIndices_(new PointIndices),
      imageCloudPoints_(imgCloudPointsFile.c_str()),
      calibFile_(calibFile)
{
    readParameters();
    pub_plane_      = nodehandle_.advertise<sensor_msgs::PointCloud2> ("/plane", 1);
    pub_pc2_cut_    = nodehandle_.advertise<sensor_msgs::PointCloud2> ("/cloudCut", 1);
    pub_imgColorPc_ = nodehandle_.advertise<sensor_msgs::PointCloud2> ("/imgColorCloud", 1);
}

void CamLaserCalib::readParameters()
{
    readCalibPara(calibFile_.c_str());
    nodehandle_.param("strSub_pc2",    strSub_pc2_,     string("/velodyne32/velodyne_points"));
    nodehandle_.param("strSub_img",    strSub_img_,     string("/camera/image_color"));
    nodehandle_.param("onlyDrawPointsColor",    onlyDrawPointsColor_,     true);
    nodehandle_.param("DistanceThreshold",      distance_threshold_,      double(0.05));                //ransac DistanceThreshold
}
void CamLaserCalib::cfgCallback(const Parameter cfg)
{
    cfg_ = cfg;
}
void CamLaserCalib::img_pc2_Callback(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    /************************** get image data **************************/
    grabImg(img);
    markCapture_.processFrame(img_);
    /******************** process point cloud *********************/
    pc2Time_ = pc2->header.stamp;
    sensor_msgs::PointCloudPtr cloud1(new sensor_msgs::PointCloud);
    sensor_msgs::convertPointCloud2ToPointCloud(*pc2, *cloud1);
    ROS2PCL(cloud1, pc_);
    //************************** draw point colud **************************//
    drawPointcloudColor(pc_,imgColorCloud_,img_,R_,T_,cameraMat_, distCoeff_,imageSize_);

    if(!onlyDrawPointsColor_)
    {
        /************************** process image **************************/
        if(!markCapture_.m_markers.size())
            return;
        getImgPoint();
        /******************** process point cloud *********************/
        //step 1: Cut Area
        subrectangle(pc_, pc_cut_,
                     cfg_.x_max,cfg_.x_min,cfg_.y_max,cfg_.y_min,cfg_.z_max,cfg_.z_min);

        //step 2: Get Plane
        GetPlane(pc_cut_,cloudPlane_,cloudPoint_,coefficients_,inlierIndices_,distance_threshold_);

        /***************************** save QR code board center points in file*****************************************/
        imageCloudPoints_<< cloudPoint_.x <<" "<< cloudPoint_.y <<" "<< cloudPoint_.z<<" "<< imgPoint_.x <<" "<<imgPoint_.y<<endl;
    }
     publishCloud();
}

void CamLaserCalib::grabImg(const sensor_msgs::ImageConstPtr& img)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img,"bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
        //Here we get the cv::Mat message ,that is cv_ptr->image
      cv_ptr->image.copyTo(img_);
}
void CamLaserCalib::getImgPoint()
{
    //get the cross point from four corner points: http://blog.csdn.net/yangtrees/article/details/7965983
    vector<Point2f> points = markCapture_.m_markers[0].m_points;
    double a0 = points[0].y- points[2].y;
    double b0 = points[2].x- points[0].x;
    double c0 = points[0].x * points[2].y - points[2].x * points[0].y;

    double a1 = points[1].y- points[3].y;
    double b1 = points[3].x- points[1].x;
    double c1 = points[1].x * points[3].y - points[3].x * points[1].y;

    double d = a0 * b1 - a1 * b0;
    imgPoint_.x = (b0 * c1 - b1 * c0)/d;
    imgPoint_.y = (a1 * c0 - a0 * c1)/d;
    circle(img_,imgPoint_, 3, Scalar(0,0,255), 2, 8);
    // show marker in image
    for(int i=0; i<markCapture_.m_markers.size(); i++)
    {
        int sizeNum = markCapture_.m_markers[i].m_points.size();
        for (int j=0; j<sizeNum; j++)
        {
            line(img_, markCapture_.m_markers[i].m_points[j], markCapture_.m_markers[i].m_points[(j+1)%sizeNum], Scalar(0,0,255), 2, 8);
        }
        circle(img_, markCapture_.m_markers[i].m_points[0], 3, Scalar(0,255,255), 2, 8);//clockwise code the points
    }
    imshow("markerDetector", img_);
    cv::waitKey(5);
}

int CamLaserCalib::readCalibPara(string filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout<<"Invalid calibration filename."<<std::endl;
        return 0;
    }
    fs[CAMERAMAT]>>cameraMat_;
    fs[DISTCOEFF]>>distCoeff_;
    fs[CAMERAEXTRINSICMAT]>>cameraExtrinsicMat_;
    fs[IMAGESIZE]>>imageSize_;
//    R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3)).t();
//    T_ = -R_*cameraExtrinsicMat_(cv::Rect(3,0,1,3));
    R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3));
    T_ = cameraExtrinsicMat_(cv::Rect(3,0,1,3));
    cout<<"cameraMat:"<<cameraMat_<<endl;
    cout<<"distCoeff:"<<distCoeff_<<endl;
    cout<<"cameraExtrinsicMat:"<<cameraExtrinsicMat_<<endl;
    cout<<"imageSize:"<<imageSize_<<endl;
}

void CamLaserCalib::publishCloud()
{
    sensor_msgs::PointCloud2 cloud_s_p;
    pcl::PCLPointCloud2 cloud_pc2;
    if(!onlyDrawPointsColor_)
    {
        //punlish estimated plane cloud
        pcl::toPCLPointCloud2(*cloudPlane_,cloud_pc2);
        pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
        cloud_s_p.header.frame_id = "velodyne32";
        cloud_s_p.header.stamp = pc2Time_;
        pub_plane_.publish(cloud_s_p);

        //punlish cut area of point cloud
        pcl::toPCLPointCloud2(*pc_cut_,cloud_pc2);
        pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
        cloud_s_p.header.frame_id = "velodyne32";
        cloud_s_p.header.stamp = pc2Time_;
        pub_pc2_cut_.publish(cloud_s_p);

    }
    //punlish image color cloud
    pcl::toPCLPointCloud2(*imgColorCloud_,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "velodyne32";
    cloud_s_p.header.stamp = pc2Time_;
    pub_imgColorPc_.publish(cloud_s_p);

}




