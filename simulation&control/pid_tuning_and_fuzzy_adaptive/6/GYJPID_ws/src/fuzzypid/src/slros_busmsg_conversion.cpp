#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_FuzzyPID_geometry_msgs_Point and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_FuzzyPID_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_FuzzyPID_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_FuzzyPID_geometry_msgs_Pose and geometry_msgs::Pose

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_FuzzyPID_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBus(&msgPtr->position, &busPtr->Position);
}

void convertToBus(SL_Bus_FuzzyPID_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBus(&busPtr->Position, &msgPtr->position);
}


// Conversions between SL_Bus_FuzzyPID_geometry_msgs_Quaternion and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_FuzzyPID_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_FuzzyPID_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_FuzzyPID_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_FuzzyPID_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->nsec =  busPtr->Nsec;
  msgPtr->sec =  busPtr->Sec;
}

void convertToBus(SL_Bus_FuzzyPID_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Nsec =  msgPtr->nsec;
  busPtr->Sec =  msgPtr->sec;
}


// Conversions between SL_Bus_FuzzyPID_sensor_msgs_Joy and sensor_msgs::Joy

void convertFromBus(sensor_msgs::Joy* msgPtr, SL_Bus_FuzzyPID_sensor_msgs_Joy const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/Joy");

  convertFromBusVariablePrimitiveArray(msgPtr->axes, busPtr->Axes, busPtr->Axes_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->buttons, busPtr->Buttons, busPtr->Buttons_SL_Info);
  convertFromBus(&msgPtr->header, &busPtr->Header);
}

void convertToBus(SL_Bus_FuzzyPID_sensor_msgs_Joy* busPtr, sensor_msgs::Joy const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/Joy");

  convertToBusVariablePrimitiveArray(busPtr->Axes, busPtr->Axes_SL_Info, msgPtr->axes, slros::EnabledWarning(rosMessageType, "axes"));
  convertToBusVariablePrimitiveArray(busPtr->Buttons, busPtr->Buttons_SL_Info, msgPtr->buttons, slros::EnabledWarning(rosMessageType, "buttons"));
  convertToBus(&busPtr->Header, &msgPtr->header);
}


// Conversions between SL_Bus_FuzzyPID_std_msgs_Float32 and std_msgs::Float32

void convertFromBus(std_msgs::Float32* msgPtr, SL_Bus_FuzzyPID_std_msgs_Float32 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float32");

  msgPtr->data =  busPtr->Data;
}

void convertToBus(SL_Bus_FuzzyPID_std_msgs_Float32* busPtr, std_msgs::Float32 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float32");

  busPtr->Data =  msgPtr->data;
}


// Conversions between SL_Bus_FuzzyPID_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_FuzzyPID_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_FuzzyPID_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_FuzzyPID_std_msgs_Int16 and std_msgs::Int16

void convertFromBus(std_msgs::Int16* msgPtr, SL_Bus_FuzzyPID_std_msgs_Int16 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Int16");

  msgPtr->data =  busPtr->Data;
}

void convertToBus(SL_Bus_FuzzyPID_std_msgs_Int16* busPtr, std_msgs::Int16 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Int16");

  busPtr->Data =  msgPtr->data;
}

