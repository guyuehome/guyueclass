#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_turtletrajectory_geometry_msgs_Twist and geometry_msgs::Twist

void convertFromBus(geometry_msgs::Twist* msgPtr, SL_Bus_turtletrajectory_geometry_msgs_Twist const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertFromBus(&msgPtr->angular, &busPtr->Angular);
  convertFromBus(&msgPtr->linear, &busPtr->Linear);
}

void convertToBus(SL_Bus_turtletrajectory_geometry_msgs_Twist* busPtr, geometry_msgs::Twist const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertToBus(&busPtr->Angular, &msgPtr->angular);
  convertToBus(&busPtr->Linear, &msgPtr->linear);
}


// Conversions between SL_Bus_turtletrajectory_geometry_msgs_Vector3 and geometry_msgs::Vector3

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_turtletrajectory_geometry_msgs_Vector3 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_turtletrajectory_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_turtletrajectory_turtlesim_Pose and turtlesim::Pose

void convertFromBus(turtlesim::Pose* msgPtr, SL_Bus_turtletrajectory_turtlesim_Pose const* busPtr)
{
  const std::string rosMessageType("turtlesim/Pose");

  msgPtr->angular_velocity =  busPtr->AngularVelocity;
  msgPtr->linear_velocity =  busPtr->LinearVelocity;
  msgPtr->theta =  busPtr->Theta;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
}

void convertToBus(SL_Bus_turtletrajectory_turtlesim_Pose* busPtr, turtlesim::Pose const* msgPtr)
{
  const std::string rosMessageType("turtlesim/Pose");

  busPtr->AngularVelocity =  msgPtr->angular_velocity;
  busPtr->LinearVelocity =  msgPtr->linear_velocity;
  busPtr->Theta =  msgPtr->theta;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
}

