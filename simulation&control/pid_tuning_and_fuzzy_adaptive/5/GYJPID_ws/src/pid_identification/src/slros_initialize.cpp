#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "PID_identification";

// For Block PID_identification/Subscribe
SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_PID_identification_sensor_msgs_Joy> Sub_PID_identification_3;

// For Block PID_identification/Publish
SimulinkPublisher<std_msgs::Int16, SL_Bus_PID_identification_std_msgs_Int16> Pub_PID_identification_2;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

