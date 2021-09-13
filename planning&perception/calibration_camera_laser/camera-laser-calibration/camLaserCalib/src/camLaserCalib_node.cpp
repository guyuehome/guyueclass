#include <ros/ros.h>
#include "camLaserCalib.h"
#include <dynamic_reconfigure/server.h>
#include <cam_laser_calib/CamLaserCalibConfig.h>
using namespace message_filters;
using namespace sensor_msgs;
using namespace std;
Parameter param;

void Callback(cam_laser_calib::CamLaserCalibConfig &config, uint32_t level)
{
    param.x_min           = config.x_min;
    param.x_max           = config.x_max;
    param.y_min           = config.y_min;
    param.y_max           = config.y_max;
    param.z_min           = config.z_min;
    param.z_max           = config.z_max;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_laser_calib");
    if(argc!=3)
    {
        cerr<<endl<<"Usage: rosrun cam_laser_calib cam_laser_calib_node path_to_pointsFile path_to_calibFile";
        ros::shutdown();
        return 1;
    }
    ros::NodeHandle nh("~");
    camLaserCalib::CamLaserCalib CamLaserCalib(nh,argv[1],argv[2]);
   // ros::spin();
    /*************************** message filter ***********************************/
    message_filters::Subscriber<PointCloud2> laser32(nh, CamLaserCalib.strSub_pc2_.c_str(), 1);
    message_filters::Subscriber<Image> img(nh,CamLaserCalib.strSub_img_.c_str() , 1);
    typedef sync_policies::ApproximateTime<Image,PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img, laser32);
    sync.registerCallback(boost::bind(&camLaserCalib::CamLaserCalib::img_pc2_Callback, &CamLaserCalib, _1, _2));
    /*************************** dynamic_reconfigure ***********************************/
    dynamic_reconfigure::Server<cam_laser_calib::CamLaserCalibConfig> server;
    dynamic_reconfigure::Server<cam_laser_calib::CamLaserCalibConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);
    ros::Rate loop(5);
    while(ros::ok())
    {
        ros::spinOnce();
        CamLaserCalib.cfgCallback(param);
    }

    return 0;
}
