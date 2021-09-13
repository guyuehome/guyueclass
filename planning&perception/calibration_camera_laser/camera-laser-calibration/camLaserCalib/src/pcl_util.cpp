#include "pcl_util.h"
using namespace cv;
void ROS2PCL(const sensor_msgs::PointCloudConstPtr packet,
             Cloud::Ptr cloud_out)
{
    //------------------Transform into pcl::pointcloud-------------------//
    sensor_msgs::PointCloud2 cloud_2;
    sensor_msgs::convertPointCloudToPointCloud2(*packet, cloud_2);

    pcl::PCLPointCloud2 cloud_pc2;
    pcl_conversions::toPCL(cloud_2, cloud_pc2);

    pcl::fromPCLPointCloud2(cloud_pc2, *cloud_out);
}

void subrectangle(Cloud::Ptr cloud_in,
                  Cloud::Ptr cloud_out,
                  double cut_max_x,
                  double cut_min_x,
                  double cut_max_y,
                  double cut_min_y,
                  double cut_max_z,
                  double cut_min_z)
{
    Cloud::Ptr cloud(new Cloud);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {

        if(cloud_in->points[i].x >= cut_min_x&&cloud_in->points[i].x <= cut_max_x&&
           cloud_in->points[i].y >= cut_min_y&&cloud_in->points[i].y <= cut_max_y&&
           cloud_in->points[i].z >= cut_min_z&&cloud_in->points[i].z <= cut_max_z)
        {
            cloud->push_back(cloud_in->points[i]);
        }
    }
    *cloud_out = *cloud;
    return;
}

bool GetPlane(Cloud::Ptr cloud_in,
              ColorCloud::Ptr cloudPlane,
              PointCT &centroid_point,
              pcl::ModelCoefficients::Ptr coeffs,
              pcl::PointIndices::Ptr indices,
              double threshold)
{

    if (cloud_in->size() <= 20)
        return EXIT_FAILURE;
    ColorCloud::Ptr cloudPlaneTemp(new ColorCloud);
    PointCT centroid_point_tmp;
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud_in);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(threshold);
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*indices,*coeffs);
   // std::cerr<<"indices->indices.size() "<<indices->indices.size()<<std::endl;
    for (std::vector<int>::const_iterator pit = indices->indices.begin(); pit != indices->indices.end(); ++pit)
    {

        PointCT point;
        point.x = cloud_in->points[*pit].x;
        point.y = cloud_in->points[*pit].y;
        point.z = cloud_in->points[*pit].z;
        point.r = 255;
        point.g = 255;
        point.b = 0;

        cloudPlaneTemp->points.push_back(point);
        centroid_point_tmp.x += cloud_in->points[*pit].x;
        centroid_point_tmp.y += cloud_in->points[*pit].y;
        centroid_point_tmp.z += cloud_in->points[*pit].z;


    }
        centroid_point_tmp.x /= indices->indices.size();
        centroid_point_tmp.y /= indices->indices.size();
        centroid_point_tmp.z /= indices->indices.size();
        centroid_point_tmp.r = 200;
        centroid_point_tmp.g = 0;
        centroid_point_tmp.b = 0;
        cloudPlaneTemp->points.push_back(centroid_point_tmp);
        centroid_point = centroid_point_tmp;
        *cloudPlane = *cloudPlaneTemp;
}
void drawPointcloudColor(Cloud::Ptr cloud_in,
                                        ColorCloud::Ptr cloud_out,
                                        const cv::Mat& img,
                                        const cv::Mat& R,
                                        const cv::Mat& T,
                                        const cv::Mat& cameraMat,
                                        const cv::Mat& distCoeff,
                                        const cv::Size& imageSize)
{
    ColorCloud::Ptr cloud_out_temp(new ColorCloud);
    PointCT drawPoint;
    int w = imageSize.width;
    int h = imageSize.height;
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        cv::Mat point(3, 1, CV_64F);
        point.at<double>(0) = cloud_in->points[i].x;
        point.at<double>(1) = -cloud_in->points[i].z;
        point.at<double>(2) = cloud_in->points[i].y;
//        point = point * invR.t() + invT.t();
        //point = invR.t()*point  + invT;
        point = R * point  + T;
        if(point.at<double>(2)>0)
        {
            double tmpx = point.at<double>(0) / point.at<double>(2);
            double tmpy = point.at<double>(1)/point.at<double>(2);
            cv::Point2d imagepoint;
    //        double r2 = tmpx * tmpx + tmpy * tmpy;
    //        double tmpdist = 1 + distCoeff.at<double>(0) * r2
    //            + distCoeff.at<double>(1) * r2 * r2
    //            + distCoeff.at<double>(4) * r2 * r2 * r2;
    //        imagepoint.x = tmpx * tmpdist
    //            + 2 * distCoeff.at<double>(2) * tmpx * tmpy
    //            + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
    //        imagepoint.y = tmpy * tmpdist
    //            + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy)
    //            + 2 * distCoeff.at<double>(3) * tmpx * tmpy;

            imagepoint.x = tmpx;
            imagepoint.y = tmpy;

            imagepoint.x = cameraMat.at<double>(0,0) * imagepoint.x + cameraMat.at<double>(0,2);
            imagepoint.y = cameraMat.at<double>(1,1) * imagepoint.y + cameraMat.at<double>(1,2);
            int px = int(imagepoint.x + 0.5);
            int py = int(imagepoint.y + 0.5);
            if(0 <= px && px < w && 0 <= py && py < h)
            {
                //int pid = py * w + px;
                drawPoint.x = cloud_in->points[i].x;
                drawPoint.y = cloud_in->points[i].y;
                drawPoint.z = cloud_in->points[i].z;
                drawPoint.b = img.at<Vec3b>(py,px)[0];
                drawPoint.g = img.at<Vec3b>(py,px)[1];
                drawPoint.r = img.at<Vec3b>(py,px)[2];
                cloud_out_temp->points.push_back(drawPoint);
            }//if cloud poind belongs to image
        }//if z>0
    }//itertor cloud
    *cloud_out = *cloud_out_temp;
}
