#ifndef _ROS_nav_msgs_GetMapFeedback_h
#define _ROS_nav_msgs_GetMapFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace nav_msgs
{

  class GetMapFeedback : public ros::Msg
  {
    public:

    GetMapFeedback()
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

    const char * getType(){ return PSTR( "nav_msgs/GetMapFeedback" ); };
    const char * getMD5(){ return PSTR( "d41d8cd98f00b204e9800998ecf8427e" ); };

  };

}
#endif