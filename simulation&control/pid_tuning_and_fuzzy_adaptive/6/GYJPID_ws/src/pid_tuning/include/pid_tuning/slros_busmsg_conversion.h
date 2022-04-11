#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include "PID_tuning_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_PID_tuning_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_PID_tuning_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_PID_tuning_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_PID_tuning_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_PID_tuning_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_PID_tuning_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_PID_tuning_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_PID_tuning_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::Joy* msgPtr, SL_Bus_PID_tuning_sensor_msgs_Joy const* busPtr);
void convertToBus(SL_Bus_PID_tuning_sensor_msgs_Joy* busPtr, sensor_msgs::Joy const* msgPtr);

void convertFromBus(std_msgs::Float32* msgPtr, SL_Bus_PID_tuning_std_msgs_Float32 const* busPtr);
void convertToBus(SL_Bus_PID_tuning_std_msgs_Float32* busPtr, std_msgs::Float32 const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_PID_tuning_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_PID_tuning_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(std_msgs::Int16* msgPtr, SL_Bus_PID_tuning_std_msgs_Int16 const* busPtr);
void convertToBus(SL_Bus_PID_tuning_std_msgs_Int16* busPtr, std_msgs::Int16 const* msgPtr);


#endif
