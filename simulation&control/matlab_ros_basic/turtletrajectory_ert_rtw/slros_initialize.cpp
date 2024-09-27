#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "turtletrajectory";

// For Block turtletrajectory/Subscribe
SimulinkSubscriber<turtlesim::Pose, SL_Bus_turtletrajectory_turtlesim_Pose> Sub_turtletrajectory_108;

// For Block turtletrajectory/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_turtletrajectory_geometry_msgs_Twist> Pub_turtletrajectory_103;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

