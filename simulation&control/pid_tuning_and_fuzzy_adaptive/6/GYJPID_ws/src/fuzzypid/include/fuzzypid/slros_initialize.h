#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "FuzzyPID_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block FuzzyPID/Subscribe
extern SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_FuzzyPID_sensor_msgs_Joy> Sub_FuzzyPID_3;

// For Block FuzzyPID/Subscribe1
extern SimulinkSubscriber<std_msgs::Float32, SL_Bus_FuzzyPID_std_msgs_Float32> Sub_FuzzyPID_18;

// For Block FuzzyPID/Publish
extern SimulinkPublisher<std_msgs::Int16, SL_Bus_FuzzyPID_std_msgs_Int16> Pub_FuzzyPID_2;

// For Block FuzzyPID/Publish1
extern SimulinkPublisher<std_msgs::Float32, SL_Bus_FuzzyPID_std_msgs_Float32> Pub_FuzzyPID_34;

// For Block FuzzyPID/Publish2
extern SimulinkPublisher<geometry_msgs::Pose, SL_Bus_FuzzyPID_geometry_msgs_Pose> Pub_FuzzyPID_37;

void slros_node_init(int argc, char** argv);

#endif
