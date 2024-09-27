#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block turtletrajectory/Subscribe
extern SimulinkSubscriber<turtlesim::Pose, SL_Bus_turtletrajectory_turtlesim_Pose> Sub_turtletrajectory_108;

// For Block turtletrajectory/Publish
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_turtletrajectory_geometry_msgs_Twist> Pub_turtletrajectory_103;

void slros_node_init(int argc, char** argv);

#endif
