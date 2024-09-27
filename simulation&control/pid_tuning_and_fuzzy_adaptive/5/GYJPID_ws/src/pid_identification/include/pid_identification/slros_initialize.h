#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "PID_identification_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block PID_identification/Subscribe
extern SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_PID_identification_sensor_msgs_Joy> Sub_PID_identification_3;

// For Block PID_identification/Publish
extern SimulinkPublisher<std_msgs::Int16, SL_Bus_PID_identification_std_msgs_Int16> Pub_PID_identification_2;

void slros_node_init(int argc, char** argv);

#endif
