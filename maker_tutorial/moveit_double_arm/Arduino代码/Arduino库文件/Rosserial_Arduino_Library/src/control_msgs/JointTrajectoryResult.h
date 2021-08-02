#ifndef _ROS_control_msgs_JointTrajectoryResult_h
#define _ROS_control_msgs_JointTrajectoryResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace control_msgs
{

  class JointTrajectoryResult : public ros::Msg
  {
    public:

    JointTrajectoryResult()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return PSTR( "control_msgs/JointTrajectoryResult" ); };
    const char * getMD5(){ return PSTR( "d41d8cd98f00b204e9800998ecf8427e" ); };

  };

}
#endif