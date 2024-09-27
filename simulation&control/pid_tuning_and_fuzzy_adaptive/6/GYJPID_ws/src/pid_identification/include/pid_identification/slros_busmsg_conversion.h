#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include "PID_identification_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(ros::Time* msgPtr, SL_Bus_PID_identification_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_PID_identification_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::Joy* msgPtr, SL_Bus_PID_identification_sensor_msgs_Joy const* busPtr);
void convertToBus(SL_Bus_PID_identification_sensor_msgs_Joy* busPtr, sensor_msgs::Joy const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_PID_identification_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_PID_identification_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(std_msgs::Int16* msgPtr, SL_Bus_PID_identification_std_msgs_Int16 const* busPtr);
void convertToBus(SL_Bus_PID_identification_std_msgs_Int16* busPtr, std_msgs::Int16 const* msgPtr);


#endif
