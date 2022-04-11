#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "FuzzyPID";

// For Block FuzzyPID/Subscribe
SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_FuzzyPID_sensor_msgs_Joy> Sub_FuzzyPID_3;

// For Block FuzzyPID/Subscribe1
SimulinkSubscriber<std_msgs::Float32, SL_Bus_FuzzyPID_std_msgs_Float32> Sub_FuzzyPID_18;

// For Block FuzzyPID/Publish
SimulinkPublisher<std_msgs::Int16, SL_Bus_FuzzyPID_std_msgs_Int16> Pub_FuzzyPID_2;

// For Block FuzzyPID/Publish1
SimulinkPublisher<std_msgs::Float32, SL_Bus_FuzzyPID_std_msgs_Float32> Pub_FuzzyPID_34;

// For Block FuzzyPID/Publish2
SimulinkPublisher<geometry_msgs::Pose, SL_Bus_FuzzyPID_geometry_msgs_Pose> Pub_FuzzyPID_37;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

